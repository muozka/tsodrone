"""
Drone Kontrol Sistemi - Görüntü İşleme Modülü. 
"""
import re
import logging
import numpy as np
import cv2
import sys
import os

# Config erişimi için path ayarı
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from config.settings import ALPR

logger = logging.getLogger(__name__)

class ALPRDetector:
    """Nesne tespiti."""

    def __init__(self):
        self._model = None
        self._reader = None
        self._target_plate = None
        self._frame_n = 0
        self._last_result = self._empty()
        # main.py'deki hatayı çözen değişken
        self.model_type = "YOLO11-ONNX" 

    # ── LAZY LOAD (Performans Odaklı Yükleme) ──────────────────
    def _load(self):
        if self._model is not None:
            return
        
        # 1. YOLO Yükleme (Önce ONNX denenir)
        try:
            from ultralytics import YOLO
            
            # Eğer .onnx dosyası varsa onu yükle, yoksa .pt üzerinden devam et
            onnx_path = ALPR.MODEL_PATH.replace(".pt", ".onnx")
            
            if os.path.exists(onnx_path):
                logger.info(f"HIZLI MOD: ONNX model yukleniyor: {onnx_path}")
                self._model = YOLO(onnx_path, task="detect")
                self.model_type = "YOLO-ONNX (Hızlı)"
            else:
                logger.info(f"STANDART MOD: PyTorch model yukleniyor: {ALPR.MODEL_PATH}")
                self._model = YOLO(ALPR.MODEL_PATH)
                self.model_type = "YOLO-PyTorch (Yavaş)"
                
            logger.info(f"YOLO hazir. Mod: {self.model_type}")
        except Exception as e:
            logger.error(f"YOLO YÜKLEME HATASI: {e}")
            self.model_type = "YOK (Hata)"

        # 2. OCR Yükleme (Sadece ihtiyaç duyulduğunda)
        try:
            import easyocr
            logger.info("EasyOCR yukleniyor (CPU optimize)...")
            # Raspberry Pi için GPU kapalı, sessiz mod açık
            self._reader = easyocr.Reader(ALPR.OCR_LANGUAGES, gpu=False, verbose=False)
            logger.info("EasyOCR hazir.")
        except Exception as e:
            logger.warning(f"EasyOCR yuklenemedi: {e}")

    # ── HEDEF KİLİTLEME ────────────────────────────────────
    def lock_plate(self, plate: str):
        if plate:
            self._target_plate = plate.upper().strip()
            logger.info(f"HEDEF BELIRLENDI: {self._target_plate}")

    def clear_target(self):
        self._target_plate = None

    # ── GÖRÜNTÜ İYİLEŞTİRME (OCR Başarısı İçin) ─────────────
    @staticmethod
    def _preprocess(roi: np.ndarray) -> np.ndarray:
        if roi is None or roi.size == 0:
            return None
        
        # Plaka çok küçükse büyüt (OCR için kritik)
        h, w = roi.shape[:2]
        if w < 120:
            roi = cv2.resize(roi, None, fx=2.0, fy=2.0, interpolation=cv2.INTER_CUBIC)
            
        gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
        # Parlama ve gölgeleri temizle (CLAHE)
        clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8))
        enh = clahe.apply(gray)
        return enh

    def _read_plate(self, roi):
        if self._reader is None: return "", 0.0
        try:
            processed_roi = self._preprocess(roi)
            # OCR tahmini
            results = self._reader.readtext(processed_roi, detail=1)
            
            if not results: return "", 0.0
            
            # En yüksek güvenli sonucu al
            best = max(results, key=lambda r: r[2])
            text = best[1].upper().replace(" ", "").replace("-", "")
            conf = float(best[2])
            return text, conf
        except Exception as e:
            logger.error(f"OCR Isleme Hatasi: {e}")
            return "", 0.0

    @staticmethod
    def _valid(text):
        if not text or len(text) < 5: return False
        # Basit Türk plakası kontrolü (Örn: 34ABC123)
        return bool(re.match(r"^[0-9]{2}[A-Z ]{1,3}[0-9]{2,4}$", text))

    # ── ANA DÖNGÜ (Tespit) ──────────────────────────────────
    def detect(self, frame: np.ndarray) -> dict:
        self._load()
        self._frame_n += 1
        
        # Performans için her X karede bir işlem yap (Config'den gelir)
        if self._frame_n % ALPR.FRAME_SKIP != 0:
            return self._last_result

        result = self._empty(frame)
        if self._model is None: return result

        try:
            # YOLO Tahmini (ONNX ile çok daha hızlı)
            dets = self._model(frame, conf=ALPR.YOLO_CONFIDENCE, verbose=False)
            
            if not dets or len(dets[0].boxes) == 0:
                self._last_result = result
                return result

            # En yüksek güvenli kutuyu (box) işle
            box = dets[0].boxes[0]
            x1, y1, x2, y2 = map(int, box.xyxy[0])
            
            # ROI kesme
            roi = frame[max(0, y1):y2, max(0, x1):x2]
            
            if roi.size > 0:
                text, score = self._read_plate(roi)
                
                # Hedef kontrolü (Belirli bir hedef yoksa gördüğü her şeyi hedef kabul et)
                is_target = False
                if self._target_plate:
                    if text == self._target_plate:
                        is_target = True
                else:
                    is_target = True

                # Görsel geri bildirim (OCR bulsa da bulmasa da kutu çizilecek)
                color = (0, 255, 0) if is_target else (0, 165, 255)
                cv2.rectangle(result["frame_annotated"], (x1, y1), (x2, y2), color, 2)
                
                if text:
                    cv2.putText(result["frame_annotated"], f"{text} ({score:.2f})", 
                                (x1, y1-10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)
                else:
                    cv2.putText(result["frame_annotated"], "NESNE", 
                                (x1, y1-10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)

                result.update({
                    "detected": True,
                    "plate_text": text if text else "BELİRSİZ",
                    "confidence": score,
                    "bbox": (x1, y1, x2, y2),
                    "is_target": is_target
                })

        except Exception as e:
            logger.error(f"Tespit dongusu hatasi: {e}")

        self._last_result = result
        return result

    @staticmethod
    def _empty(frame=None):
        return {
            "detected": False, 
            "plate_text": "", 
            "confidence": 0.0,
            "bbox": None, 
            "is_target": False,
            "frame_annotated": frame.copy() if frame is not None else None
        }