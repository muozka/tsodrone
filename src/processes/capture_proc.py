"""
src/processes/capture_proc.py

Kamera yakalama process'i.

PiCamera2 (varsa) veya OpenCV ile frame yakalar.
JPEG olarak encode eder, frame_queue'ya koyar.
Hedef: 25-30 FPS yakalama, 4-5 FPS YOLO'ya gönderme.

Frame boyutu: Camera.WIDTH x Camera.HEIGHT
JPEG kalite: düşük (YOLO için, bant genişliği tasarrufu)
"""
import os
import sys
import time
import logging
import multiprocessing as mp

ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.insert(0, ROOT)


def capture_process_main(
    frame_queue: mp.Queue,      # YOLO process'e JPEG frame + timestamp
    mjpeg_queue: mp.Queue,      # ws_server'a ham JPEG (görüntü yayını için)
    log_queue:   mp.Queue,
    stop_event:  mp.Event,
):
    """
    Kamera yakalama process'i.
    frame_queue: {"jpeg": bytes, "ts": float, "frame_id": int}
    mjpeg_queue: JPEG bytes (sadece akış için — ws_server okur)
    """
    _setup_proc_logging(log_queue, "capture_proc")
    logger = logging.getLogger("capture_proc")

    from config.settings import Camera, ALPR

    cap = _open_camera(Camera, logger)
    if cap is None:
        logger.error("[CAP] Kamera açılamadı!")
        return

    logger.info(f"[CAP] Kamera açıldı: {Camera.WIDTH}x{Camera.HEIGHT}@{Camera.FPS}FPS")

    frame_id  = 0
    yolo_skip = 0
    # YOLO'ya her kaç capture'de bir gönder
    # Hedef YOLO FPS: ALPR.YOLO_MAX_FPS (5)
    # Capture FPS: Camera.FPS (30)
    yolo_every = max(1, Camera.FPS // ALPR.YOLO_MAX_FPS)  # 30//5 = 6

    last_fps_log = time.monotonic()
    fps_count    = 0

    try:
        while not stop_event.is_set():
            t_start = time.monotonic()

            # Frame yakala
            ret, frame = _read_frame(cap, Camera, logger)
            if not ret or frame is None:
                time.sleep(0.01)
                continue

            frame_id  += 1
            fps_count += 1

            # FPS logu (her 10s)
            now = time.monotonic()
            if now - last_fps_log >= 10.0:
                fps = fps_count / (now - last_fps_log)
                logger.info(f"[CAP] Yakalama FPS: {fps:.1f}")
                fps_count    = 0
                last_fps_log = now

            # MJPEG stream için (ws_server — her kare)
            try:
                import cv2
                ok, mjpeg = cv2.imencode(
                    ".jpg", frame,
                    [cv2.IMWRITE_JPEG_QUALITY, 55]
                )
                if ok:
                    jbytes = mjpeg.tobytes()
                    # Non-blocking — eski kare atılır
                    try:
                        mjpeg_queue.put_nowait(jbytes)
                    except Exception:
                        try:
                            mjpeg_queue.get_nowait()
                            mjpeg_queue.put_nowait(jbytes)
                        except Exception:
                            pass
            except Exception as e:
                logger.debug(f"[CAP] MJPEG encode hatası: {e}")

            # YOLO'ya gönderilecek kare (416x416, düşük kalite)
            yolo_skip += 1
            if yolo_skip >= yolo_every:
                yolo_skip = 0
                try:
                    import cv2
                    from config.settings import ALPR as _A
                    small = cv2.resize(
                        frame,
                        (_A.INFER_SIZE, _A.INFER_SIZE),
                        interpolation=cv2.INTER_LINEAR,
                    )
                    ok, yjpeg = cv2.imencode(
                        ".jpg", small,
                        [cv2.IMWRITE_JPEG_QUALITY, 75]
                    )
                    if ok:
                        item = {
                            "jpeg":     yjpeg.tobytes(),
                            "ts":       time.monotonic(),
                            "frame_id": frame_id,
                            "orig_w":   Camera.WIDTH,
                            "orig_h":   Camera.HEIGHT,
                        }
                        try:
                            frame_queue.put_nowait(item)
                        except Exception:
                            try:
                                frame_queue.get_nowait()
                                frame_queue.put_nowait(item)
                            except Exception:
                                pass
                except Exception as e:
                    logger.debug(f"[CAP] YOLO encode hatası: {e}")

            # FPS kontrolü
            elapsed = time.monotonic() - t_start
            wait    = max(0.0, (1.0 / Camera.FPS) - elapsed)
            if wait > 0:
                time.sleep(wait)

    finally:
        _close_camera(cap)
        logger.info("[CAP] Kapatıldı")


# ── Kamera açma/kapama (PiCamera2 / OpenCV) ───────────────────

def _open_camera(cfg, logger):
    """PiCamera2 önce dene, sonra OpenCV."""
    # Önce PiCamera2
    try:
        from picamera2 import Picamera2
        cam = Picamera2()
        config = cam.create_video_configuration(
            main={"size": (cfg.WIDTH, cfg.HEIGHT), "format": "RGB888"},
            controls={"FrameRate": cfg.FPS},
        )
        cam.configure(config)
        cam.start()
        logger.info("[CAP] PiCamera2 açıldı")
        return ("picamera2", cam)
    except Exception as e:
        logger.info(f"[CAP] PiCamera2 yok ({e}), OpenCV deneniyor...")

    # OpenCV fallback
    try:
        import cv2
        cap = cv2.VideoCapture(cfg.INDEX)
        if cap.isOpened():
            cap.set(cv2.CAP_PROP_FRAME_WIDTH,  cfg.WIDTH)
            cap.set(cv2.CAP_PROP_FRAME_HEIGHT, cfg.HEIGHT)
            cap.set(cv2.CAP_PROP_FPS,          cfg.FPS)
            logger.info("[CAP] OpenCV kamera açıldı")
            return ("opencv", cap)
    except Exception as e:
        logger.error(f"[CAP] OpenCV hatası: {e}")

    return None


def _read_frame(cap_tuple, cfg, logger):
    kind, cap = cap_tuple
    try:
        if kind == "picamera2":
            import numpy as np
            arr = cap.capture_array("main")
            # RGB → BGR (OpenCV format)
            import cv2
            frame = cv2.cvtColor(arr, cv2.COLOR_RGB2BGR)
            return True, frame
        else:
            return cap.read()
    except Exception as e:
        logger.debug(f"[CAP] Frame okuma hatası: {e}")
        return False, None


def _close_camera(cap_tuple):
    if cap_tuple is None:
        return
    kind, cap = cap_tuple
    try:
        if kind == "picamera2":
            cap.stop()
        else:
            cap.release()
    except Exception:
        pass


def _setup_proc_logging(log_queue: mp.Queue, proc_name: str):
    from logging.handlers import QueueHandler
    root = logging.getLogger()
    root.handlers.clear()
    root.addHandler(QueueHandler(log_queue))
    root.setLevel(logging.DEBUG)
