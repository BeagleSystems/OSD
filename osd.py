#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
osd.py — Crisp, Two-Row MAVLink OSD (FINAL FIX: AS Detection Label)
- **NEW FIELD:** 'Sats' data is now labeled as 'AS Det' (Placeholder for Gas Sensor/AS Detection).
- **COLOR:** Labels are BRIGHT YELLOW, Values/Numbers are BRIGHT WHITE.
- **EASY CONTROLS:** Large Drag (⠿) and Close (✕) buttons centered at the top.
- Two rows of telemetry:
    Row 1: Clb, GS, AS Det, Home
    Row 2: Alt(Rel), Alt(AMSL), V/Cell, Level
"""

import sys, time, math
from datetime import timedelta
from threading import Thread, Event
from PyQt5 import QtCore, QtGui, QtWidgets
from pymavlink import mavutil

# ---------- CONFIG ----------
DEFAULT_ENDPOINT   = "udpin:0.0.0.0:14551"
DEFAULT_FONT_SIZE  = 20
DEFAULT_SCREEN     = 0
DEFAULT_ANCHOR     = "top"
DEFAULT_MARGIN_PX  = 80
# --- IMPORTANT: Set your battery cell count for accurate V/Cell ---
DEFAULT_CELL_COUNT = 4 
# -----------------------------

QtCore.QCoreApplication.setAttribute(QtCore.Qt.AA_EnableHighDpiScaling, True)
QtCore.QCoreApplication.setAttribute(QtCore.Qt.AA_UseHighDpiPixmaps, True)

try:
    from geographiclib.geodesic import Geodesic
    GEOD = Geodesic.WGS84
except Exception:
    GEOD = None

def haversine(lat1, lon1, lat2, lon2):
    import math as m
    R = 6371000.0
    p1, p2 = m.radians(lat1), m.radians(lat2)
    dp = p2 - p1
    dl = m.radians(lon2 - lon1)
    a = m.sin(dp/2)**2 + m.cos(p1)*m.cos(p2)*m.sin(dl/2)**2
    return 2*R*m.atan2(m.sqrt(a), m.sqrt(1-a))

class TelemetryState:
    def __init__(self):
        self.lock = QtCore.QReadWriteLock()
        self.groundspeed = None; self.throttle = None; self.climb = None
        self.alt_amsl = None; self.alt_rel = None
        self.sats = None; self.batt_voltage_total = None; self.batt_remaining = None
        self.armed = False; self.armed_since = None
        self.home = None; self.latlon = None
        self.roll = None; self.pitch = None 
        self.batt_cells = DEFAULT_CELL_COUNT 

    def update(self, **kw):
        self.lock.lockForWrite()
        for k,v in kw.items(): setattr(self, k, v)
        self.lock.unlock()
    def snapshot(self):
        self.lock.lockForRead()
        snap = dict(vars(self))
        self.lock.unlock()
        return snap

class MavReader(Thread):
    def __init__(self, endpoint, state, stop):
        super().__init__(daemon=True)
        self.endpoint = endpoint; self.state = state; self.stop = stop
        self.m = None
        self.last_heartbeat_sent = 0

    def connect(self):
        self.m = mavutil.mavlink_connection(self.endpoint, autoreconnect=True)
        try: self.m.wait_heartbeat(timeout=5)
        except: pass
        
    def send_heartbeat(self):
        """Sends a heartbeat message to the autopilot to prompt it to send data."""
        if not self.m: return
        
        self.m.mav.heartbeat_send(
            mavutil.mavlink.MAV_TYPE_GCS,
            mavutil.mavlink.MAV_AUTOPILOT_INVALID,
            0, 0,
            mavutil.mavlink.MAV_STATE_ACTIVE
        )
        self.last_heartbeat_sent = time.time()

    def request_streams(self):
        """Sends MAVLink commands to request frequent data stream updates."""
        if not self.m or not self.m.target_system:
            print("MAVLink connection not ready to request streams.")
            return

        system_id = self.m.target_system
        component_id = self.m.target_component
        
        # MAVLink Message IDs to request and their intervals in microseconds
        messages = [
            (mavutil.mavlink.MAVLINK_MSG_ID_HEARTBEAT, 1000000), 
            (mavutil.mavlink.MAVLINK_MSG_ID_ATTITUDE, 100000), 
            (mavutil.mavlink.MAVLINK_MSG_ID_VFR_HUD, 100000), 
            (mavutil.mavlink.MAVLINK_MSG_ID_GLOBAL_POSITION_INT, 1000000), 
            (mavutil.mavlink.MAVLINK_MSG_ID_GPS_RAW_INT, 1000000), 
            (mavutil.mavlink.MAVLINK_MSG_ID_SYS_STATUS, 1000000),
        ]
        
        for msg_id, interval_us in messages:
            self.m.mav.command_long_send(
                system_id, component_id,
                mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
                0, msg_id, interval_us, 0, 0, 0, 0, 0
            )
        
        self.m.mav.command_long_send(
            system_id, component_id,
            mavutil.mavlink.MAV_CMD_GET_HOME_POSITION,
            0, 0, 0, 0, 0, 0, 0, 0
        )
        print("Requested data streams from autopilot.")

    def run(self):
        self.connect()
        
        try:
            if self.m.wait_heartbeat(timeout=5):
                self.request_streams()
            else:
                print("No heartbeat received from MAVLink source.")
        except Exception as e:
            print(f"Error during initial connection/stream request: {e}")
            pass

        while not self.stop.is_set():
            if time.time() - self.last_heartbeat_sent > 1:
                self.send_heartbeat()
                
            try: msg = self.m.recv_match(blocking=True, timeout=0.1)
            except: msg = None
            if not msg: continue
            
            t = msg.get_type()
            if t == "HEARTBEAT":
                armed = bool(getattr(msg,"base_mode",0) & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
                snap = self.state.snapshot()
                if armed and not snap.get("armed"):
                    self.state.update(armed=True, armed_since=time.time())
                elif not armed and snap.get("armed"):
                    self.state.update(armed=False)
                else:
                    self.state.update(armed=armed)
            elif t == "VFR_HUD":
                self.state.update(alt_amsl=msg.alt, groundspeed=msg.groundspeed,
                                   climb=msg.climb, throttle=msg.throttle)
            elif t == "GLOBAL_POSITION_INT":
                if msg.relative_alt is not None: self.state.update(alt_rel=msg.relative_alt/1000.0)
                self.state.update(latlon=(msg.lat/1e7, msg.lon/1e7))
            elif t == "GPS_RAW_INT":
                self.state.update(sats=msg.satellites_visible)
            elif t == "SYS_STATUS":
                if msg.voltage_battery>0: self.state.update(batt_voltage_total=msg.voltage_battery/1000.0)
                if msg.battery_remaining!=255: self.state.update(batt_remaining=msg.battery_remaining)
            elif t == "BATTERY_STATUS":
                if hasattr(msg,"voltages"):
                    vs=[v for v in msg.voltages if v and v<65535]
                    if vs: self.state.update(batt_voltage_total=sum(vs)/1000.0)
            elif t == "HOME_POSITION":
                self.state.update(home=(msg.latitude/1e7,msg.longitude/1e7,(msg.altitude or 0)/1000.0))
            elif t == "ATTITUDE":
                self.state.update(roll=math.degrees(msg.roll), pitch=math.degrees(msg.pitch))

class OSD(QtWidgets.QWidget):
    def __init__(self, state, font_size=DEFAULT_FONT_SIZE):
        super().__init__()
        self.setWindowFlags(QtCore.Qt.FramelessWindowHint|QtCore.Qt.WindowStaysOnTopHint|
                            QtCore.Qt.Tool|QtCore.Qt.X11BypassWindowManagerHint)
        self.setAttribute(QtCore.Qt.WA_TranslucentBackground, True)
        self.state = state

        f = QtGui.QFont("DejaVu Sans Mono", font_size)
        f.setBold(False)
        self.font = f

        self.row1 = QtWidgets.QLabel(); self.row2 = QtWidgets.QLabel()
        LABEL_STYLE = "QLabel { background-color: rgba(0,0,0,0.4); border-radius: 4px; padding: 1px 4px; }"
        for lab in (self.row1,self.row2):
            lab.setFont(f); lab.setTextFormat(QtCore.Qt.RichText)
            lab.setStyleSheet(LABEL_STYLE)
            lab.setAlignment(QtCore.Qt.AlignCenter) 

        LARGE_ICON_SIZE = 40
        icon_font = QtGui.QFont("Sans", LARGE_ICON_SIZE)
        
        self.dragBtn = QtWidgets.QLabel("⠿")
        self.dragBtn.setFont(icon_font)
        self.dragBtn.setStyleSheet("QLabel {color:#FFFF00; background:rgba(0,0,0,0.4); border-radius: 4px; padding: 0 5px;}") 
        
        self.closeBtn = QtWidgets.QLabel("✕")
        self.closeBtn.setFont(icon_font)
        self.closeBtn.setStyleSheet("QLabel {color:#FF0000; background:rgba(0,0,0,0.4); border-radius: 4px; padding: 0 5px;}") 

        self.controlsBar = QtWidgets.QHBoxLayout()
        self.controlsBar.addStretch(1) 
        self.controlsBar.addWidget(self.dragBtn)
        self.controlsBar.addSpacing(10)
        self.controlsBar.addWidget(self.closeBtn)
        self.controlsBar.addStretch(1) 

        vbox = QtWidgets.QVBoxLayout()
        vbox.setContentsMargins(0,0,0,0); vbox.setSpacing(5)
        vbox.addLayout(self.controlsBar) 
        vbox.addWidget(self.row1)
        vbox.addWidget(self.row2)
        
        self.setLayout(vbox)

        self.dragging = False
        self.offset = QtCore.QPoint()

        self.closeBtn.mousePressEvent = self.close_clicked
        self.dragBtn.mousePressEvent = self.start_drag
        self.dragBtn.mouseMoveEvent  = self.do_drag
        self.dragBtn.mouseReleaseEvent = self.stop_drag

        self.timer = QtCore.QTimer(self)
        self.timer.timeout.connect(self.refresh)
        self.timer.start(100)

    def close_clicked(self, e): self.close()

    def start_drag(self, e):
        if e.button()==QtCore.Qt.LeftButton:
            self.dragging=True
            self.offset = e.globalPos() - self.frameGeometry().topLeft()

    def do_drag(self, e):
        if self.dragging:
            self.move(e.globalPos()-self.offset)

    def stop_drag(self, e):
        if e.button()==QtCore.Qt.LeftButton: self.dragging=False

    # helpers
    def pad_value(self, v, width):
        if v is None: return "—".ljust(width)
        s = str(v)
        return s.rjust(width).replace(' ', '&nbsp;')

    def f(self,v,fmt,width,dash="—"):
        try: 
            if v is None: return dash.ljust(width).replace(' ', '&nbsp;')
            s = fmt.format(v)
            return s.rjust(width).replace(' ', '&nbsp;')
        except: return dash.ljust(width).replace(' ', '&nbsp;')

    def dist_home(self,s):
        if not s.get("home") or not s.get("latlon"): return None
        (lat1,lon1)=s["home"][:2]; (lat2,lon2)=s["latlon"]
        if GEOD: return GEOD.Inverse(lat1,lon1,lat2,lon2)["s12"]
        return haversine(lat1,lon1,lat2,lon2)
    def avg_cell_voltage(self,s):
        if s.get("batt_voltage_total") and s.get("batt_cells") and s["batt_cells"] > 0:
            return s["batt_voltage_total"] / s["batt_cells"]
        return None
    
    # Labels are BRIGHT YELLOW, Values are BRIGHT WHITE
    def span(self,label,value,unit="",lc="#FFFF00",vc="#FFFFFF"):
        if value.strip() == "—":
            return f'<span style="color:{lc}">{label} <span style="color:#9E9E9E">{value}</span>{unit}</span>'
        return f'<span style="color:{lc}">{label}&nbsp;<span style="color:{vc}">{value}</span>{unit}</span>'

    def refresh(self):
        s=self.state.snapshot()
        d=self.dist_home(s)

        # ----------------------------------------------------
        # Telemetry Data with Fixed Width Formatting
        # ----------------------------------------------------
        clb=self.f(s.get("climb"),"{:4.1f}", 4)             # Climbrate 
        gs=self.f(s.get("groundspeed"),"{:4.1f}", 4)        # Ground speed 
        alt_rel=self.f(s.get("alt_rel"),"{:5.1f}", 5)       # Alt(Relative)
        alt_amsl=self.f(s.get("alt_amsl"),"{:5.1f}", 5)     # Alt(AMSL)
        v_cell=self.f(self.avg_cell_voltage(s),"{:3.2f}", 4)# Average cell voltage 
        # MODIFICATION: Use Sats data but label it as AS Det
        sats=self.f(s.get("sats"),"{:2.0f}", 2)             
        
        # Combined Horizontal Level (Roll / Pitch)
        roll=self.f(s.get("roll"),"{:0.0f}", 3, dash="—")  
        pitch=self.f(s.get("pitch"),"{:0.0f}", 3, dash="—") 
        level_value = f'{roll}&nbsp;/&nbsp;{pitch}'
        
        # Home distance 
        home_val = "{:5.1f}".format(d) if d is not None else "—"
        home=self.pad_value(home_val, 5)

        # Supporting data
        bat_rem=self.f(s.get("batt_remaining"),"{:3.0f}", 3) 
        # ----------------------------------------------------

        
        # Row 1: Clb, GS, AS Det, Home
        self.row1.setText(" &nbsp;&nbsp;&nbsp; ".join([
            self.span("Clb",clb,"m/s"),
            self.span("GS",gs,"m/s"),
            self.span("AS Det",sats,""), # Renamed 'Sats' label to 'AS Det'
            self.span("Home",home,"m")
        ]))
        
        # Row 2: Alt(Rel), Alt(AMSL), V/C, Level (Roll/Pitch)
        self.row2.setText(" &nbsp;&nbsp;&nbsp; ".join([
            self.span("Alt(Rel)",alt_rel,"m"),
            self.span("Alt(AMSL)",alt_amsl,"m"),
            self.span("V/C",v_cell,"V"),
            self.span("Level",level_value,"°")
        ]))
        self.adjustSize()

class App(QtWidgets.QApplication):
    def __init__(self):
        super().__init__(sys.argv)
        self.setApplicationName("OSD")
        self.state = TelemetryState()
        self.stop = Event()
        self.reader = MavReader(DEFAULT_ENDPOINT,self.state,self.stop)
        self.reader.start()
        self.w = OSD(self.state,DEFAULT_FONT_SIZE)
        self.position(DEFAULT_SCREEN,DEFAULT_ANCHOR,DEFAULT_MARGIN_PX)
        self.w.show()
        self.aboutToQuit.connect(self.cleanup)
    def cleanup(self):
        self.stop.set()
        if self.reader.is_alive(): self.reader.join(timeout=1.0)
    def position(self,screen,anchor,margin):
        geo=self.screens()[screen].geometry()
        self.w.refresh()
        w,h=self.w.width(),self.w.height()
        if anchor=="top": x=geo.x()+(geo.width()-w)//2; y=geo.y()+margin
        elif anchor=="bottom": x=geo.x()+(geo.width()-w)//2; y=geo.y()+geo.height()-h-margin
        else: x=geo.x()+(geo.width()-w)//2; y=geo.y()+margin
        self.w.move(x,y)

def main():
    app=App()
    sys.exit(app.exec_())

if __name__=="__main__":
    main()
