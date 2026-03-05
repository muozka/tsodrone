"""
src/processes/yolo_proc.py

YOLO inference process — ONNX INT8, 416x416, max 5 FPS.

Her N karede bir çalışır; aralarında tracking kullanılır (OCR proc).
Tespit edilen bbox'ları detection_queue'ya koyar.

INT8 ONNX dönüşümü (ilk kurulumda bir kez yapılır):
    python3 scripts/export_int8.py
    (bkz. scripts/export_int8.py)
"""
import os
import sys
import time
import logging
import multiprocessing as mp

ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.insert(0, ROOT)


def yolo_process_main(
    frame_queue:     mp.Queue,     # capture_proc'dan JPEG kareler
    detection_queue: mp.Queue,     # detections → pursuit_controller
    ocr_queue:       mp.Queue,     # ROI crops → ocr_proc
    log_queue:       mp.Queue,
    stop_event:      mp.Event,
):
    """
    YOLO process giriş noktası.
    detection_queue item:
      {
        "bbox":     (x1,y1,x2,y2),   # orijinal frame koordinatlarına scale edilmiş
        "conf":     float,
        "ts":       float,            # tespitın zamanı
        "frame_id": int,
        "roi_jpeg": bytes,            # OCR için crop (opsiyonel)
      }
    None = hedef yok
    """
    _setup_proc_logging(log_queue, "yolo_proc")
    logger = logging.getLogger("yolo_proc")
    logger.info("[YOLO] Process başladı")

    from config.settings import ALPR

    model = _load_model(ALPR, logger)

    fps_count    = 0
    last_fps_log = time.monotonic()
    last_infer   = time.monotonic()
    min_interval = 1.0 / ALPR.YOLO_MAX_FPS   # 5 FPS → 200ms

    logger.info(f"[YOLO] Hazır — max {ALPR.YOLO_MAX_FPS} FPS, "
                f"giriş boyutu {ALPR.INFER_SIZE}x{ALPR.INFER_SIZE}")

    while not stop_event.is_set():
        # Hız sınırlama — YOLO_MAX_FPS aşma
        now = time.monotonic()
        since_last = now - last_infer
        if since_last < min_interval:
            time.sleep(min_interval - since_last)
            continue

        # Frame al (blocking 1s timeout)
        try:
            item = frame_queue.get(timeout=1.0)
        except Exception:
            # Timeout — "hedef yok" yayınla
            _push_no_detection(detection_queue)
            continue

        if item is None:
            break

        jpeg_bytes = item["jpeg"]
        orig_w     = item.get("orig_w", 640)
        orig_h     = item.get("orig_h", 480)
        ts         = item["ts"]
        frame_id   = item["frame_id"]

        last_infer = time.monotonic()
        fps_count += 1

        # FPS logu (her 10s)
        if time.monotonic() - last_fps_log >= 10.0:
            fps = fps_count / (time.monotonic() - last_fps_log)
            logger.info(f"[YOLO] Inference FPS: {fps:.1f}")
            fps_count    = 0
            last_fps_log = time.monotonic()

        # JPEG → numpy
        try:
            import cv2, numpy as np
            nparr = np.frombuffer(jpeg_bytes, np.uint8)
            frame = cv2.imdecode(nparr, cv2.IMREAD_COLOR)
            if frame is None:
                _push_no_detection(detection_queue)
                continue
        except Exception as e:
            logger.debug(f"[YOLO] Decode hatası: {e}")
            _push_no_detection(detection_queue)
            continue

        # ── YOLO Inference ──
        best_bbox = None
        best_conf = 0.0
        best_roi  = None

        if model is not None:
            try:
                from config.settings import ALPR as _A
                dets = model(frame, conf=_A.YOLO_CONFIDENCE, verbose=False)

                if dets and len(dets[0].boxes) > 0:
                    # En yüksek confidence'lı kutu
                    boxes = dets[0].boxes
                    confs = boxes.conf.cpu().numpy()
                    idx   = int(confs.argmax())
                    conf  = float(confs[idx])

                    x1, y1, x2, y2 = map(int, boxes.xyxy[idx].cpu().numpy())

                    # YOLO giriş 416x416 → orijinal boyuta scale et
                    sx = orig_w / _A.INFER_SIZE
                    sy = orig_h / _A.INFER_SIZE
                    x1 = int(x1 * sx); y1 = int(y1 * sy)
                    x2 = int(x2 * sx); y2 = int(y2 * sy)
                    # Sınır kontrolü
                    x1 = max(0, x1); y1 = max(0, y1)
                    x2 = min(orig_w - 1, x2); y2 = min(orig_h - 1, y2)

                    best_bbox = (x1, y1, x2, y2)
                    best_conf = conf

                    # OCR için ROI JPEG
                    try:
                        roi = frame[y1:y2, x1:x2]
                        if roi.size > 0:
                            # Küçük ROI'yi büyüt
                            rh, rw = roi.shape[:2]
                            if rw < 120:
                                roi = cv2.resize(roi, None, fx=2.0, fy=2.0,
                                                 interpolation=cv2.INTER_CUBIC)
                            ok, roi_j = cv2.imencode(
                                ".jpg", roi,
                                [cv2.IMWRITE_JPEG_QUALITY, 90]
                            )
                            best_roi = roi_j.tobytes() if ok else None
                    except Exception:
                        best_roi = None

            except Exception as e:
                logger.warning(f"[YOLO] Inference hatası: {e}")

        # ── Sonucu detection_queue'ya koy ──
        if best_bbox is not None:
            det = {
                "bbox":     best_bbox,
                "conf":     best_conf,
                "ts":       ts,
                "frame_id": frame_id,
                "detected": True,
            }
            # OCR kuyruğuna ROI gönder
            if best_roi is not None:
                try:
                    ocr_item = {
                        "roi_jpeg": best_roi,
                        "bbox":     best_bbox,
                        "frame_id": frame_id,
                        "ts":       ts,
                    }
                    ocr_queue.put_nowait(ocr_item)
                except Exception:
                    pass
        else:
            det = {"detected": False, "ts": ts, "frame_id": frame_id}

        try:
            detection_queue.put_nowait(det)
        except Exception:
            try:
                detection_queue.get_nowait()
                detection_queue.put_nowait(det)
            except Exception:
                pass

    logger.info("[YOLO] Process kapatıldı")


def _push_no_detection(q: mp.Queue):
    try:
        q.put_nowait({"detected": False, "ts": time.monotonic()})
    except Exception:
        pass


def _load_model(cfg, logger):
    """
    YOLO modeli yükle.
    Öncelik: INT8 ONNX → FP32 ONNX → PT model
    """
    import os

    # Model yolu varyantları
    base = cfg.MODEL_PATH.replace(".onnx", "").replace(".pt", "")
    candidates = [
        base + "_int8.onnx",    # INT8 ONNX (en hızlı)
        base + ".onnx",          # FP32 ONNX
        base + ".pt",            # PyTorch (en yavaş)
        cfg.MODEL_PATH,          # config'deki yol
    ]

    for path in candidates:
        if not os.path.exists(path):
            continue
        try:
            from ultralytics import YOLO
            model = YOLO(path, task="detect")
            # Warmup
            import numpy as np
            dummy = np.zeros((cfg.INFER_SIZE, cfg.INFER_SIZE, 3), dtype=np.uint8)
            model(dummy, conf=cfg.YOLO_CONFIDENCE, verbose=False)
            logger.info(f"[YOLO] Model yüklendi: {path}")
            return model
        except Exception as e:
            logger.warning(f"[YOLO] {path} yüklenemedi: {e}")
            continue

    logger.error("[YOLO] Model bulunamadı — tespit devre dışı")
    return None


def _setup_proc_logging(log_queue: mp.Queue, proc_name: str):
    from logging.handlers import QueueHandler
    root = logging.getLogger()
    root.handlers.clear()
    root.addHandler(QueueHandler(log_queue))
    root.setLevel(logging.DEBUG)
