"""
=======================================================================
  DRONE PLAKA TAKİP SİSTEMİ — Multi-Process Mimari
  src/main.py
=======================================================================

Process mimarisi:
  P1  MAVLink    — Pixhawk 2.4.8 iletişim + RTL watchdog
  P2  Capture    — Kamera yakalama (30 FPS)
  P3  YOLO       — INT8 ONNX inference (max 5 FPS, 416x416)
  P4  OCR        — EasyOCR (cooldown + confidence eşiği)
  P0  Main       — Takip kontrolü, ws_server, süreç yönetimi

IPC:
  command_queue   : Main → MAVLink      (drone komutları)
  telemetry_queue : MAVLink → Main      (telemetri snapshotu)
  frame_queue     : Capture → YOLO      (JPEG kareler)
  mjpeg_queue     : Capture → Main      (ws_server için ham akış)
  detection_queue : YOLO → Main         (bbox detections)
  ocr_queue       : YOLO → OCR          (ROI crops)
  ocr_result_queue: OCR → Main          (tanınan plakalar)
  log_queue       : All → Main          (merkezi loglama)
  watchdog_ts     : Main → MAVLink      (shared Value, heartbeat)
"""
import argparse
import ctypes
import json
import logging
import logging.handlers
import multiprocessing as mp
import os
import signal
import socket
import sys
import threading
import time
from datetime import datetime
from pathlib import Path

ROOT = Path(__file__).parent.parent
sys.path.insert(0, str(ROOT))

from config.settings import (
    MAVLink as MAVCfg, Flight, WebSocket as WSCfg,
    GCS as GCSCfg, FailSafe, Log, Camera, ALPR, Pursuit,
)

# ══════════════════════════════════════════════════════════════════
#  LOGLAMA — merkezi + her process QueueHandler kullanır
# ══════════════════════════════════════════════════════════════════
def _setup_main_logging(log_queue: mp.Queue):
    log_dir = ROOT / Log.DIR
    log_dir.mkdir(exist_ok=True)
    ts = datetime.now().strftime("%Y%m%d_%H%M%S")

    handlers = [
        logging.StreamHandler(sys.stdout),
        logging.handlers.RotatingFileHandler(
            log_dir / f"flight_{ts}.log",
            maxBytes=Log.ROTATE_MB * 1024 * 1024,
            backupCount=3,
            encoding="utf-8",
        ),
    ]
    logging.basicConfig(
        level=getattr(logging, Log.LEVEL, logging.INFO),
        format="%(asctime)s [%(levelname)-8s] %(name)-18s: %(message)s",
        handlers=handlers,
    )

    # Alt process'lerin loglarını dinle
    def _listener():
        while True:
            try:
                rec = log_queue.get(timeout=1.0)
                if rec is None:
                    break
                logging.getLogger(rec.name).handle(rec)
            except Exception:
                pass

    t = threading.Thread(target=_listener, daemon=True, name="LogListener")
    t.start()
    return t


# ══════════════════════════════════════════════════════════════════
#  GCS UDP
# ══════════════════════════════════════════════════════════════════
class GCSSender:
    def __init__(self):
        self._sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._addr = (GCSCfg.IP, GCSCfg.PORT)

    def send_alert(self, plate, lat, lon, alt, conf):
        try:
            payload = json.dumps({
                "event": "plate_detected", "plate": plate,
                "lat": lat, "lon": lon, "alt": alt,
                "confidence": round(conf, 3),
                "ts": datetime.utcnow().isoformat(),
            }).encode()
            self._sock.sendto(payload, self._addr)
        except Exception:
            pass

    def close(self):
        try:
            self._sock.close()
        except Exception:
            pass


# ══════════════════════════════════════════════════════════════════
#  ANA SİSTEM
# ══════════════════════════════════════════════════════════════════
class PlatePursuitSystem:

    def __init__(self, args):
        self.args     = args
        self._running = True
        self._gcs     = GCSSender()

        # ── IPC Kuyrukları ──────────────────────────────────────
        self.command_queue    = mp.Queue(maxsize=50)
        self.telemetry_queue  = mp.Queue(maxsize=5)
        self.frame_queue      = mp.Queue(maxsize=3)
        self.mjpeg_queue      = mp.Queue(maxsize=2)
        self.detection_queue  = mp.Queue(maxsize=10)
        self.ocr_queue        = mp.Queue(maxsize=ALPR.OCR_QUEUE_MAXSIZE)
        self.ocr_result_queue = mp.Queue(maxsize=20)
        self.log_queue        = mp.Queue(maxsize=500)

        # ── Watchdog timestamp (shared Value) ───────────────────
        self.watchdog_ts = mp.Value(ctypes.c_double, 0.0)

        # ── Paylaşılan telemetri state (thread-safe dict) ────────
        # ws_server ve pursuit_controller buradan okur
        self._telem_state: dict = {
            "lat": 0.0, "lon": 0.0, "alt": 0.0, "heading": 0.0,
            "speed": 0.0, "battery": -1.0, "voltage": 0.0,
            "mode": "UNKNOWN", "mode_num": -1, "armed": False,
            "gps_fix": 0, "satellites": 0, "ekf_ok": False,
            "bat_ready": False, "mission_time": 0,
        }
        self._telem_lock = threading.Lock()

        self._processes: list[mp.Process] = []

    # ── BAŞLAT ────────────────────────────────────────────────────
    def start(self):
        logger = logging.getLogger("main")
        logger.info("=" * 60)
        logger.info("  DRONE PLAKA TAKİP — Multi-Process Başlıyor")
        logger.info("=" * 60)

        if self.args.sitl:
            os.environ["MAVLINK_URI"] = "tcp:127.0.0.1:5760"
            logger.info("[MAIN] SITL modu — tcp:127.0.0.1:5760")

        # ── P1: MAVLink Process ──────────────────────────────────
        if not self.args.demo:
            from src.processes.mavlink_proc import mavlink_process_main
            p_mav = mp.Process(
                target=mavlink_process_main,
                args=(
                    self.command_queue,
                    self.telemetry_queue,
                    self.log_queue,
                    self.watchdog_ts,
                    MAVCfg.CONNECTION_STRING,
                    MAVCfg.BAUD_RATE,
                ),
                name="MAVLinkProc",
                daemon=True,
            )
            p_mav.start()
            self._processes.append(p_mav)
            logger.info(f"[MAIN] MAVLink process başlatıldı (PID={p_mav.pid})")
            time.sleep(2.0)   # Pixhawk bağlantısını bekle

        # ── P2: Capture Process ──────────────────────────────────
        stop_event = mp.Event()
        from src.processes.capture_proc import capture_process_main
        p_cap = mp.Process(
            target=capture_process_main,
            args=(
                self.frame_queue,
                self.mjpeg_queue,
                self.log_queue,
                stop_event,
            ),
            name="CaptureProc",
            daemon=True,
        )
        p_cap.start()
        self._processes.append(p_cap)
        self._stop_event = stop_event
        logger.info(f"[MAIN] Capture process başlatıldı (PID={p_cap.pid})")

        # ── P3: YOLO Process ─────────────────────────────────────
        from src.processes.yolo_proc import yolo_process_main
        p_yolo = mp.Process(
            target=yolo_process_main,
            args=(
                self.frame_queue,
                self.detection_queue,
                self.ocr_queue,
                self.log_queue,
                stop_event,
            ),
            name="YOLOProc",
            daemon=True,
        )
        p_yolo.start()
        self._processes.append(p_yolo)
        logger.info(f"[MAIN] YOLO process başlatıldı (PID={p_yolo.pid})")

        # ── P4: OCR Process ──────────────────────────────────────
        from src.processes.ocr_proc import ocr_process_main
        p_ocr = mp.Process(
            target=ocr_process_main,
            args=(
                self.ocr_queue,
                self.ocr_result_queue,
                self.log_queue,
                stop_event,
            ),
            name="OCRProc",
            daemon=True,
        )
        p_ocr.start()
        self._processes.append(p_ocr)
        logger.info(f"[MAIN] OCR process başlatıldı (PID={p_ocr.pid})")

        # ── ws_server kurulumu ───────────────────────────────────
        import src.ws_server as ws_server
        self._ws = ws_server

        if not self.args.demo:
            from src.drone_proxy import DroneProxy
            proxy = DroneProxy(self.command_queue, self._telem_state)
            ws_server.inject_drone(proxy)
        else:
            ws_server._state.arm()
            logger.info("[MAIN] Demo modu — proxy yok")

        # ── ws_server thread'de başlat ───────────────────────────
        self._start_ws_server(ws_server)

        # ── Telemetri güncelleme thread'i ────────────────────────
        threading.Thread(
            target=self._telem_reader,
            daemon=True,
            name="TelemReader",
        ).start()

        # ── Watchdog kalp atışı thread'i ─────────────────────────
        if not self.args.demo:
            threading.Thread(
                target=self._watchdog_heartbeat,
                daemon=True,
                name="WatchdogHB",
            ).start()

        # ── Kalkış ───────────────────────────────────────────────
        if not self.args.demo:
            time.sleep(3.0)   # Process'lerin hazırlanmasını bekle
            logger.info("[MAIN] Kalkış komutu gönderiliyor...")
            self.command_queue.put({"cmd": "takeoff", "altitude": Flight.TARGET_ALTITUDE_M})
        else:
            logger.info("[MAIN] Demo modu — kalkış atlandı")

        # ── Ana görev döngüsü ────────────────────────────────────
        logger.info("[MAIN] Görev döngüsü başlıyor...")
        from src.pursuit_controller import PursuitController
        pursuit = PursuitController(self.command_queue)

        try:
            self._mission_loop(pursuit, ws_server)
        except KeyboardInterrupt:
            logger.info("[MAIN] Ctrl+C — kapatılıyor...")
        finally:
            self._shutdown()

    # ── TELEMETRİ OKUYUCU ────────────────────────────────────────
    def _telem_reader(self):
        """telemetry_queue'dan oku, _telem_state'i güncelle, ws_server'ı besle."""
        logger = logging.getLogger("main.telem")
        import src.ws_server as ws_server
        from config.settings import FailSafe as FS

        warned = set()

        while self._running:
            try:
                snap = self.telemetry_queue.get(timeout=0.5)
                if not snap or "error" in snap:
                    continue

                with self._telem_lock:
                    self._telem_state.update(snap)

                # ws_server state sync (demo bölümüne girmeden)
                ws_state = ws_server._state
                ws_state.mode    = snap.get("mode", "UNKNOWN")
                ws_state.armed   = snap.get("armed", False)

                # Pil fail-safe (main process'ten)
                if snap.get("bat_ready"):
                    bat = snap.get("battery", 100.0)
                    if bat <= FS.BATTERY_LAND_PCT and "land" not in warned:
                        logger.critical(f"[MAIN] KRİTİK PİL {bat:.0f}% → LAND")
                        warned.add("land")
                        self.command_queue.put({"cmd": "land"})
                    elif bat <= FS.BATTERY_RTL_PCT and "rtl" not in warned:
                        logger.error(f"[MAIN] DÜŞÜK PİL {bat:.0f}% → RTL")
                        warned.add("rtl")
                        self.command_queue.put({"cmd": "rtl"})
                    elif bat <= FS.BATTERY_WARN_PCT and "warn" not in warned:
                        logger.warning(f"[MAIN] Pil uyarısı: {bat:.0f}%")
                        warned.add("warn")

            except Exception:
                pass

    # ── WATCHDOG KALP ATIŞI ───────────────────────────────────────
    def _watchdog_heartbeat(self):
        """MAVLink process'e düzenli heartbeat gönder."""
        while self._running:
            try:
                self.command_queue.put_nowait({"cmd": "heartbeat"})
            except Exception:
                pass
            time.sleep(0.5)

    # ── GÖREV DÖNGÜSÜ ────────────────────────────────────────────
    def _mission_loop(self, pursuit, ws_server):
        logger    = logging.getLogger("main.loop")
        last_gcs  = 0.0
        miss_count = 0
        tracking   = False
        target_plate = None

        # MJPEG → ws_server bridge thread
        threading.Thread(
            target=self._mjpeg_bridge,
            args=(ws_server,),
            daemon=True,
            name="MJPEGBridge",
        ).start()

        while self._running:
            t_start = time.monotonic()

            # OCR sonuçlarını oku
            try:
                while True:
                    res = self.ocr_result_queue.get_nowait()
                    plate = res.get("plate", "")
                    conf  = res.get("conf", 0.0)
                    if plate:
                        logger.info(f"[MAIN] Plaka tanındı: {plate} ({conf:.2f})")
                        ws_server._state.detected_plate = plate
                        ws_server._state.plate_conf     = conf
                        ws_server._state._log(f"Plaka: {plate} ({conf:.2f})", "SUCCESS")
                        # Hedef kilitli değilse otomatik kilitle
                        if not ws_server._state.target_plate and not self.args.demo:
                            ws_server._state.set_target(plate)
                        target_plate = plate
            except Exception:
                pass

            # Detection sonuçlarını oku
            det = None
            try:
                det = self.detection_queue.get_nowait()
            except Exception:
                pass

            # Manuel mod kontrolü
            manual_mode = (ws_server._state.mode == "MANUEL")

            if det and not manual_mode:
                detected = det.get("detected", False)
                bbox     = det.get("bbox") if detected else None
                det_ts   = det.get("ts", time.monotonic())

                if bbox:
                    miss_count = 0
                    tracking   = True
                    ws_server._state.tracking = True

                    # Hedef plate kontrolü
                    tp = ws_server._state.target_plate
                    if tp and target_plate and tp != target_plate:
                        # Farklı plaka — takip etme
                        logger.debug(f"[MAIN] Yanlış plaka, atlıyorum ({target_plate} ≠ {tp})")
                        bbox = None

                    if bbox:
                        pursuit.update(bbox, Camera.WIDTH, Camera.HEIGHT, det_ts)

                    # GCS
                    now = time.time()
                    if now - last_gcs >= GCSCfg.SEND_INTERVAL:
                        last_gcs = now
                        with self._telem_lock:
                            t = dict(self._telem_state)
                        self._gcs.send_alert(
                            target_plate or "HEDEF",
                            t.get("lat", 0), t.get("lon", 0),
                            t.get("alt", 0), det.get("conf", 0)
                        )
                else:
                    miss_count += 1
                    if miss_count == 1:
                        self.command_queue.put({"cmd": "velocity", "vx": 0, "vy": 0, "vz": 0, "yaw_rate": 0})
                    if miss_count >= Pursuit.MAX_MISS_FRAMES:
                        tracking = False
                        ws_server._state.tracking = False
                        pursuit.reset()
                        miss_count = 0
                        logger.info("[MAIN] Hedef kayboldu — devriye")

            # Döngü frekans kontrolü (CPU tasarrufu)
            elapsed = time.monotonic() - t_start
            sleep_t = max(0.005, 0.033 - elapsed)   # ~30 Hz
            time.sleep(sleep_t)

    # ── MJPEG KÖPRÜSÜ ────────────────────────────────────────────
    def _mjpeg_bridge(self, ws_server):
        """mjpeg_queue → ws_server._cam_frame (manuel kontrol arayüzüne)."""
        while self._running:
            try:
                jpeg = self.mjpeg_queue.get(timeout=0.5)
                with ws_server._cam_lock:
                    ws_server._cam_frame = jpeg
            except Exception:
                pass

    # ── WS SERVER ────────────────────────────────────────────────
    def _start_ws_server(self, ws_server):
        import uvicorn
        def _run():
            uvicorn.run(
                ws_server.app,
                host=WSCfg.HOST,
                port=WSCfg.PORT,
                log_level="error",
                limit_concurrency=15,
            )
        threading.Thread(target=_run, daemon=True, name="WsServer").start()
        logger = logging.getLogger("main")
        logger.info(f"[MAIN] WebSocket sunucusu: http://0.0.0.0:{WSCfg.PORT}")

    # ── KAPATMA ──────────────────────────────────────────────────
    def _shutdown(self):
        logger = logging.getLogger("main")
        logger.info("[MAIN] Kapatılıyor...")
        self._running = False

        # Tüm process'leri durdur
        try:
            self._stop_event.set()
        except Exception:
            pass

        # MAVLink'e RTL gönder
        try:
            self.command_queue.put_nowait({"cmd": "rtl"})
            time.sleep(1.0)
            self.command_queue.put_nowait(None)   # kapatma sinyali
        except Exception:
            pass

        for p in self._processes:
            try:
                p.join(timeout=3.0)
                if p.is_alive():
                    p.terminate()
            except Exception:
                pass

        self._gcs.close()
        logger.info("[MAIN] Sistem kapatıldı.")


# ══════════════════════════════════════════════════════════════════
#  ARGÜMANLAR
# ══════════════════════════════════════════════════════════════════
def _parse_args():
    p = argparse.ArgumentParser(description="Drone Plaka Takip Sistemi")
    p.add_argument("--sitl",    action="store_true", help="SITL simülasyon modu")
    p.add_argument("--demo",    action="store_true", help="Drone olmadan test")
    p.add_argument("--display", action="store_true", help="Ekran çıktısı (debug)")
    return p.parse_args()


# ══════════════════════════════════════════════════════════════════
#  GİRİŞ NOKTASI
# ══════════════════════════════════════════════════════════════════
if __name__ == "__main__":
    # Raspberry Pi'de spawn method (fork yerine) daha güvenli
    mp.set_start_method("spawn", force=True)

    args   = _parse_args()
    log_q  = mp.Queue(maxsize=500)
    log_t  = _setup_main_logging(log_q)

    logger = logging.getLogger("main")

    system = PlatePursuitSystem(args)
    # Doğru log_queue'yu enjekte et
    system.log_queue = log_q

    # SIGTERM handler
    def _sigterm(sig, frame):
        logger.info("[MAIN] SIGTERM alındı")
        system._running = False
        system._shutdown()
        sys.exit(0)

    signal.signal(signal.SIGTERM, _sigterm)

    try:
        system.start()
    except KeyboardInterrupt:
        system._shutdown()
    finally:
        log_q.put(None)   # log listener'ı kapat
