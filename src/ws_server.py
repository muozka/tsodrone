"""
Drone Kontrol Sistemi — WebSocket GCS Sunucusu
src/ws_server.py  |  Python 3.13 uyumlu
"""
import asyncio, json, time, math, random, threading, logging
from datetime import datetime
from pathlib import Path
from fastapi.responses import StreamingResponse
# --- LOGGING YAPILANDIRMASI (Hataları görmek için şart) ---
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s [%(levelname)-8s] %(name)s: %(message)s'
)
logger = logging.getLogger("ws_server")

from fastapi import FastAPI, WebSocket, WebSocketDisconnect
from fastapi.staticfiles import StaticFiles
from fastapi.responses import FileResponse, JSONResponse
from fastapi.middleware.cors import CORSMiddleware

import sys, os
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from config.settings import WebSocket as WS

app = FastAPI(title="Drone GCS API", version="2.0")
app.add_middleware(CORSMiddleware, allow_origins=["*"],
                   allow_methods=["*"], allow_headers=["*"])

# Statik dosyalar (GCS HTML arayuzu)
_STATIC = Path(__file__).parent.parent / "static"
if _STATIC.exists():
    app.mount("/static", StaticFiles(directory=str(_STATIC)), name="static")


# ─────────────────────────────────────────────────────────────
#  BAGLANTI YONETICISI
# ─────────────────────────────────────────────────────────────
class _ConnManager:
    def __init__(self):
        self._clients: list[WebSocket] = []
        self._lock = asyncio.Lock()

    async def connect(self, ws: WebSocket):
        await ws.accept()
        async with self._lock:
            self._clients.append(ws)
        print(f"[WS] +{ws.client} | toplam={len(self._clients)}")

    async def disconnect(self, ws: WebSocket):
        async with self._lock:
            if ws in self._clients:
                self._clients.remove(ws)
        print(f"[WS] -{ws.client} | toplam={len(self._clients)}")

    async def broadcast(self, data: dict):
        msg  = json.dumps(data, ensure_ascii=False)
        dead = []
        async with self._lock:
            targets = list(self._clients)
        for ws in targets:
            try:
                await ws.send_text(msg)
            except Exception:
                dead.append(ws)
        async with self._lock:
            for ws in dead:
                if ws in self._clients:
                    self._clients.remove(ws)

    @property
    def count(self) -> int:
        return len(self._clients)

_mgr = _ConnManager()


# ─────────────────────────────────────────────────────────────
#  SIMULE DURUM (demo / gercek drone yokken)
# ─────────────────────────────────────────────────────────────
class _SimState:
    def __init__(self):
        self.lat = 41.6781; self.lon = 27.4350
        self.alt = 0.0;     self.heading = 0.0
        self.speed = 0.0;   self.battery = 95.0; self.voltage = 16.4
        self.mode = "BEKLEMEDE"; self.armed = False
        self.satellites = 12; self.gps_fix = 3; self.signal = 87
        self.target_plate = None; self.detected_plate = None
        self.plate_conf = 0.0;  self.tracking = False
        self.mission_time = 0;  self._t0 = None; self._t = 0.0
        self._logs: list[dict] = []

    def tick(self):
        self._t += 0.04
        if self.armed:
            self.lat     += math.sin(self._t * 0.3) * 0.000014
            self.lon     += math.cos(self._t * 0.2) * 0.000014
            self.heading  = (self.heading + 1.1) % 360
            self.speed    = 4.2 + math.sin(self._t) * 1.1
            self.alt      = 15 + math.sin(self._t * 0.5) * 0.3 if self.alt >= 14.5 else self.alt + 0.4
            self.battery  = max(0, self.battery - 0.004)
            self.voltage  = 16.8 - (95 - self.battery) * 0.04
            if self._t0: self.mission_time = int(time.time() - self._t0)
            if self.target_plate and random.random() < 0.05:
                self.detected_plate = self.target_plate
                self.plate_conf     = round(0.78 + random.random() * 0.2, 3)
                self.tracking       = True
                self.mode           = "TAKIP"
                self._log("HEDEF TESPIT EDILDI", "SUCCESS")
            elif self.tracking and random.random() < 0.025:
                self.tracking = False; self.detected_plate = None
                self.mode = "DEVRIYE"
                self._log("Hedef gorustan cikti — DEVRIYE")
        else:
            self.alt = 0.0; self.speed = 0.0

    def arm(self):
        self.armed = True; self.mode = "GUIDED"; self._t0 = time.time()
        self._log("ARM edildi — kalkis hazir", "SUCCESS")

    def disarm(self):
        self.armed = False; self.mode = "BEKLEMEDE"; self.tracking = False
        self._log("DISARM edildi", "WARN")

    def set_target(self, plate: str):
        self.target_plate = plate.upper().strip()
        self._log("Hedef kilitlendi", "SUCCESS")

    def clear_target(self):
        self.target_plate = None; self.detected_plate = None; self.tracking = False
        self._log("Hedef temizlendi")

    def rtl(self):
        self.mode = "RTL"; self.tracking = False; self._log("RTL komutu", "WARN")

    def land(self):
        self.mode = "LAND"; self._log("LAND komutu", "WARN")

    def _log(self, msg: str, level: str = "INFO"):
        self._logs.append({"time": datetime.now().strftime("%H:%M:%S"), "level": level, "msg": msg})
        if len(self._logs) > 150: self._logs.pop(0)

    def to_dict(self) -> dict:
        return {
            "type": "telemetry", "ts": datetime.now().isoformat(),
            "lat":  round(self.lat, 6), "lon": round(self.lon, 6),
            "alt":  round(self.alt, 1), "heading": round(self.heading, 1),
            "speed": round(self.speed, 2), "battery": round(self.battery, 1),
            "voltage": round(self.voltage, 2),
            "mode": self.mode, "armed": self.armed,
            "gps_fix": self.gps_fix, "satellites": self.satellites,
            "signal": self.signal,
            "target_plate": self.target_plate,
            "detected_plate": self.detected_plate,
            "plate_conf":     self.plate_conf,
            "tracking":       self.tracking,
            "mission_time":   self.mission_time,
            "cam_fps":        _cam_fps,
        }

_state = _SimState()
_real_drone = None


# ─────────────────────────────────────────────────────────────
#  GERCEK DRONE ENTEGRASYONU
# ─────────────────────────────────────────────────────────────
def inject_drone(drone):
    global _real_drone
    _real_drone = drone
    print("[GCS] Gercek Pixhawk baglantisi aktif.")


def _get_telemetry() -> dict:
    if _real_drone and _real_drone.connected:
        snap = _real_drone.telemetry.copy()
        snap.update({
            "type":           "telemetry",
            "ts":             datetime.now().isoformat(),
            "target_plate":   _state.target_plate,
            "detected_plate": _state.detected_plate,
            "plate_conf":     _state.plate_conf,
            "tracking":       _state.tracking,
            "signal":         _state.signal,
            "cam_fps":        _cam_fps,
        })
        return snap
    _state.tick()
    return _state.to_dict()


# ─────────────────────────────────────────────────────────────
#  TELEMETRI YAYINI
# ─────────────────────────────────────────────────────────────
async def _broadcast_loop():
    interval = 1.0 / WS.TELEMETRY_HZ
    while True:
        if _mgr.count > 0:
            await _mgr.broadcast(_get_telemetry())
            while _state._logs:
                entry = _state._logs.pop(0)
                await _mgr.broadcast({"type": "log", **entry})
        await asyncio.sleep(interval)


@app.on_event("startup")
async def _on_startup():
    asyncio.create_task(_broadcast_loop())
    print(f"[GCS] Sunucu hazir: http://0.0.0.0:{WS.PORT}/")
    print(f"[GCS] WebSocket  : ws://0.0.0.0:{WS.PORT}/ws")


# ─────────────────────────────────────────────────────────────
#  HTTP ENDPOINT'LER
# ─────────────────────────────────────────────────────────────
@app.get("/")
async def root():
    idx = _STATIC / "index.html"
    if idx.exists():
        return FileResponse(str(idx))
    return JSONResponse({"status": "GCS calisiyor",
                         "ws": f"ws://[IP]:{WS.PORT}/ws",
                         "ui": "static/index.html bulunamadi"})


# ─────────────────────────────────────────────────────────────
# PICAMERA2 MJPEG STREAM — Düzeltilmiş Bölüm
# ─────────────────────────────────────────────────────────────
import io, threading as _th
_cam_res     = (640, 480)
_cam_fps     = 60
_cam_lock    = _th.Lock()
_cam_frame   = None
_ai_frame    = None
_cam_thread  = None
_cam_running = False

def _get_fps_for_res(w, h):
    # Camera 3 için optimize FPS değerleri
    if w >= 2304: return 30  # HDR / Yüksek Çözünürlük
    if w >= 1920: return 50
    if w >= 1280: return 60
    return 60 # 640x480 için 60 ideal (akıcılık vs bandwidth)

import cv2

def _start_camera():
    global _cam_frame, _cam_running, _cam_res, _cam_fps
    from picamera2 import Picamera2
    
    while _cam_running:
        cam = None
        try:
            w, h = _cam_res
            _cam_fps = _get_fps_for_res(w, h)
            
            logger.info(f"Kamera hazirlaniyor: {w}x{h} @ {_cam_fps} FPS")
            tuning = Picamera2.load_tuning_file("imx708_noir.json")
            cam = Picamera2(tuning=tuning)
            cfg = cam.create_video_configuration(
                main={"size": (w, h), "format": "RGB888"},
                controls={"FrameRate": _cam_fps},
            )
            cam.set_controls({"ExposureTime": 10000, "AnalogueGain": 1.0, "ColourGains": (2.522, 1.897)})
            cam.configure(cfg)
            cam.start()
            logger.info("Kamera basariyla baslatildi.")

            while _cam_running:
                # Çözünürlük değişmişse döngüden çıkıp kamerayı yeniden başlat
                if _cam_res != (w, h):
                    logger.info("Cozunurluk degisimi, kamera yeniden baslatiliyor...")
                    break

                arr = cam.capture_array()
                if arr is not None:
                    ok, buf = cv2.imencode(".jpg", arr, [cv2.IMWRITE_JPEG_QUALITY, 70])
                    if ok:
                        with _cam_lock:
                            _cam_frame = buf.tobytes()
                time.sleep(0.01)

        except Exception as e:
            logger.error(f"Kamera hatası: {e}")
            time.sleep(2) # Hata durumunda bekle
        finally:
            if cam:
                try:
                    cam.stop()
                    cam.close()
                except:
                    pass
                time.sleep(0.5) # Donanımın sıfırlanması için bekle

def _try_start_cam():
    global _cam_thread, _cam_running
    if _cam_thread and _cam_thread.is_alive():
        return
    _cam_running = True
    _cam_thread = _th.Thread(target=_start_camera, daemon=True, name="CamThread")
    _cam_thread.start()

async def _mjpeg_gen():
    _try_start_cam()
    while True:
        with _cam_lock:
            frame = _ai_frame if _ai_frame else _cam_frame
        if frame:
            yield (
                b"--frame\r\n"
                b"Content-Type: image/jpeg\r\n\r\n" +
                frame +
                b"\r\n"
            )
        # Çözünürlüğe göre stream hızını ayarla
        await asyncio.sleep(1.0 / _cam_fps)

@app.get("/camera/stream")
async def camera_stream():
    return StreamingResponse(
        _mjpeg_gen(),
        media_type="multipart/x-mixed-replace; boundary=frame",
        headers={
            "Cache-Control": "no-cache, no-store",
            "Pragma":         "no-cache",
            "X-Accel-Buffering": "no",
        }
    )

@app.get("/health")
async def health():
    return {
        "status":  "ok",
        "clients": _mgr.count,
        "drone":   "gercek" if (_real_drone and _real_drone.connected) else "demo",
        "mode":    _state.mode,
    }

# ─────────────────────────────────────────────────────────────
#  WEBSOCKET ENDPOINT
# ─────────────────────────────────────────────────────────────
@app.websocket("/ws")
async def ws_endpoint(websocket: WebSocket):
    await _mgr.connect(websocket)
    await websocket.send_text(json.dumps(_get_telemetry(), ensure_ascii=False))
    try:
        while True:
            raw = await websocket.receive_text()
            try:
                cmd = json.loads(raw)
                await _handle_command(cmd)
            except json.JSONDecodeError:
                pass
    except WebSocketDisconnect:
        await _mgr.disconnect(websocket)


async def _handle_command(cmd: dict):
    global _cam_res
    action = cmd.get("action", "")
    if   action == "arm":          _state.arm()
    elif action == "disarm":       _state.disarm()
    elif action == "rtl":          _state.rtl()
    elif action == "land":         _state.land()
    elif action == "set_target":
        plate = cmd.get("plate", "").strip()
        if plate:
            _state.set_target(plate)
    elif action == "clear_target": _state.clear_target()
    elif action == "change_res":
        w = cmd.get("width", 640)
        h = cmd.get("height", 480)
        _cam_res = (w, h)
    elif action == "set_mode":
        mode = cmd.get("mode", "")
        _state.mode = mode
        _state._log(f"Mod Degistirildi: {mode}")
        # Gercek drone'a ArduCopter modunu ilet
        if _real_drone and _real_drone.connected:
            mode_upper = mode.upper()
            if mode_upper in ("MANUEL", "GUIDED", "MANUAL"):
                _real_drone._conn.set_mode_guided()
            elif mode_upper in ("OTOMATIK", "AUTO", "LOITER"):
                _real_drone._conn.set_mode_loiter()
            elif mode_upper == "STABILIZE":
                _real_drone._conn.set_mode_stabilize()
            elif mode_upper in ("RTL", "RETURN"):
                _real_drone._conn.set_mode_rtl()
            elif mode_upper == "LAND":
                _real_drone._conn.set_mode_land()

    elif action == "manual_move":
        vx       = float(cmd.get("vx", 0))
        vy       = float(cmd.get("vy", 0))
        vz       = float(cmd.get("vz", 0))
        yaw_rate = float(cmd.get("yaw_rate", 0))

        if _real_drone and _real_drone.connected:
            # ArduCopter: manual hiz komutu GUIDED modda calisir.
            # Yaw rate varsa birlesik komut gonder (PX4 OFFBOARD yok).
            from src.mavlink_connection import COPTER_MODE_GUIDED
            if _real_drone._conn.mode_num != COPTER_MODE_GUIDED:
                _real_drone._conn.set_mode_guided()
            if yaw_rate != 0:
                _real_drone._conn.send_ned_velocity_with_yaw_rate(
                    vx, vy, vz, yaw_rate
                )
            else:
                _real_drone._conn.send_ned_velocity(vx, vy, vz)
    
    # Gercek drone'a ilet
    if _real_drone and _real_drone.connected:
        if action == "arm":
            _real_drone._conn.arm()
            _state._log("ARM komutu gonderildi", "WARN")
        elif action == "disarm":
            _real_drone.disarm()
        elif action == "takeoff":
            _real_drone.arm_and_takeoff()
        elif action == "rtl":
            _real_drone.return_to_launch()
        elif action == "land":
            _real_drone.land()
        elif action == "motor_test":
            throttle = float(cmd.get("throttle", 15))
            duration = float(cmd.get("duration", 3))
            motor_id = int(cmd.get("motor_id", 0))
            if not _real_drone._conn.armed:
                _real_drone._conn.arm(force=True)
                import time as _t; _t.sleep(0.5)
            _real_drone._conn.motor_test(motor_id, throttle, duration)
            _state._log(f"Motor {motor_id+1} test: %{throttle} guc, {duration}s", "WARN")
        elif action == "rc_throttle":
            pwm = int(cmd.get("pwm", 1000))
            if not _real_drone._conn.armed:
                _real_drone._conn.arm(force=True)
                import time as _t; _t.sleep(1)
            _real_drone._conn.rc_override_throttle(pwm)
            _state._log(f"RC Throttle Override: PWM={pwm}", "WARN")
        elif action == "rc_release":
            _real_drone._conn.rc_override_release()
            _state._log("RC Override serbest", "INFO")

    await _mgr.broadcast(_get_telemetry())