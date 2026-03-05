"""
src/processes/ocr_proc.py

OCR process — plaka tanıma, cooldown ve confidence eşiği ile.

Cooldown mantığı:
  Aynı plaka OCR_COOLDOWN_SEC süre içinde tekrar işlenmez.
  Bu, Raspberry Pi'nin CPU'sunu OCR ile boğmayı önler.
  EasyOCR çok ağır olduğundan, yalnızca gerektiğinde çalışır.

Confidence eşiği:
  OCR sonucu OCR_MIN_SCORE altında ise kabul edilmez.
  Belirsiz sonuçlar sessizce atılır.
"""
import os
import sys
import re
import time
import logging
import multiprocessing as mp

ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.insert(0, ROOT)


def ocr_process_main(
    ocr_queue:    mp.Queue,    # yolo_proc'dan ROI kareler
    result_queue: mp.Queue,    # tanınan plakalar → main process
    log_queue:    mp.Queue,
    stop_event:   mp.Event,
):
    """
    OCR process giriş noktası.

    ocr_queue item:
      {"roi_jpeg": bytes, "bbox": tuple, "frame_id": int, "ts": float}

    result_queue item:
      {"plate": str, "conf": float, "bbox": tuple, "ts": float}
    """
    _setup_proc_logging(log_queue, "ocr_proc")
    logger = logging.getLogger("ocr_proc")
    logger.info("[OCR] Process başladı — EasyOCR yükleniyor...")

    from config.settings import ALPR

    reader = _load_ocr(ALPR, logger)

    # Cooldown durumu: plate_text → son OCR zamanı
    cooldown: dict[str, float] = {}
    # Cooldown olmayan (henüz tanınmamış) ROI'ler için son OCR zamanı
    last_any_ocr = 0.0

    ocr_count = 0
    logger.info("[OCR] Hazır — cooldown={:.1f}s, min_conf={:.2f}".format(
        ALPR.OCR_COOLDOWN_SEC, ALPR.OCR_MIN_SCORE
    ))

    while not stop_event.is_set():
        try:
            item = ocr_queue.get(timeout=1.0)
        except Exception:
            continue

        if item is None:
            break

        roi_jpeg = item.get("roi_jpeg")
        bbox     = item.get("bbox")
        ts       = item.get("ts", time.monotonic())

        if not roi_jpeg:
            continue

        # Global cooldown: çok sık OCR çalıştırma
        now = time.monotonic()
        if now - last_any_ocr < ALPR.OCR_COOLDOWN_SEC * 0.5:
            continue   # yarı cooldown — yoğun tespit durumunda yükü azalt
        last_any_ocr = now

        if reader is None:
            continue

        # ROI decode
        try:
            import cv2, numpy as np
            nparr = np.frombuffer(roi_jpeg, np.uint8)
            roi   = cv2.imdecode(nparr, cv2.IMREAD_COLOR)
            if roi is None or roi.size == 0:
                continue
        except Exception as e:
            logger.debug(f"[OCR] ROI decode hatası: {e}")
            continue

        # Görüntü ön işleme (OCR başarısı için)
        try:
            import cv2, numpy as np
            roi_proc = _preprocess_roi(roi)
        except Exception:
            roi_proc = roi

        # EasyOCR
        try:
            results = reader.readtext(roi_proc, detail=1)
        except Exception as e:
            logger.warning(f"[OCR] readtext hatası: {e}")
            continue

        if not results:
            continue

        # En yüksek confidence'lı sonuç
        best = max(results, key=lambda r: r[2])
        raw_text = best[1].upper().replace(" ", "").replace("-", "").replace(".", "")
        conf     = float(best[2])

        logger.debug(f"[OCR] Ham: '{raw_text}' conf={conf:.2f}")

        # Confidence eşiği
        if conf < ALPR.OCR_MIN_SCORE:
            logger.debug(f"[OCR] Düşük confidence atlandı: {conf:.2f} < {ALPR.OCR_MIN_SCORE}")
            continue

        # Format doğrulama (Türk plakası)
        if not _valid_plate(raw_text):
            logger.debug(f"[OCR] Geçersiz format atlandı: '{raw_text}'")
            continue

        # Cooldown kontrolü
        last_time = cooldown.get(raw_text, 0.0)
        if now - last_time < ALPR.OCR_COOLDOWN_SEC:
            logger.debug(
                f"[OCR] Cooldown aktif: '{raw_text}' "
                f"({now - last_time:.1f}s < {ALPR.OCR_COOLDOWN_SEC}s)"
            )
            continue

        # Yeni plaka tespiti
        cooldown[raw_text] = now
        ocr_count += 1

        result = {
            "plate": raw_text,
            "conf":  conf,
            "bbox":  bbox,
            "ts":    ts,
        }

        logger.info(f"[OCR] Plaka tespit edildi: {raw_text} ({conf:.2f}) — #{ocr_count}")

        try:
            result_queue.put_nowait(result)
        except Exception:
            try:
                result_queue.get_nowait()
                result_queue.put_nowait(result)
            except Exception:
                pass

    logger.info("[OCR] Process kapatıldı")


# ── Yardımcı fonksiyonlar ────────────────────────────────────────

def _load_ocr(cfg, logger):
    try:
        import easyocr
        reader = easyocr.Reader(
            cfg.OCR_LANGUAGES,
            gpu=False,
            verbose=False,
        )
        logger.info("[OCR] EasyOCR yüklendi")
        return reader
    except Exception as e:
        logger.error(f"[OCR] EasyOCR yüklenemedi: {e}")
        return None


def _preprocess_roi(roi):
    """OCR başarısını artıran görüntü ön işleme."""
    import cv2, numpy as np
    h, w = roi.shape[:2]
    # Çok küçük ROI → büyüt
    if w < 120:
        roi = cv2.resize(roi, None, fx=2.5, fy=2.5, interpolation=cv2.INTER_CUBIC)
    elif w < 200:
        roi = cv2.resize(roi, None, fx=1.5, fy=1.5, interpolation=cv2.INTER_CUBIC)

    gray  = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
    clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8))
    enh   = clahe.apply(gray)
    # Gürültü azaltma
    blur  = cv2.bilateralFilter(enh, 9, 75, 75)
    return blur


def _valid_plate(text: str) -> bool:
    """Türk plaka formatı kontrolü: 34ABC1234"""
    if not text or len(text) < 5 or len(text) > 9:
        return False
    return bool(re.match(r"^[0-9]{2}[A-Z]{1,3}[0-9]{2,4}$", text))


def _setup_proc_logging(log_queue: mp.Queue, proc_name: str):
    from logging.handlers import QueueHandler
    root = logging.getLogger()
    root.handlers.clear()
    root.addHandler(QueueHandler(log_queue))
    root.setLevel(logging.DEBUG)
