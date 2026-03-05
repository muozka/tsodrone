"""
Drone Kontrol Sistemi — Merkezi Ayarlar
config/settings.py  |  Pixhawk 2.4.8 / ArduCopter / Multi-Process
"""
import os

# ══════════════════════════════════════════════════════════════════
#  MAVLink / Seri Bağlantı
# ══════════════════════════════════════════════════════════════════
class MAVLink:
    CONNECTION_STRING  = os.getenv("MAVLINK_URI",  "/dev/ttyAMA0")
    BAUD_RATE          = int(os.getenv("MAVLINK_BAUD", "57600"))
    TIMEOUT_SEC        = 30
    HEARTBEAT_TIMEOUT  = 5

    # Telemetri stream Hz (57600 baud sınırı için optimize)
    STREAM_SENSORS_HZ  = 10
    STREAM_POSITION_HZ = 10
    STREAM_STATUS_HZ   = 10
    STREAM_ATTITUDE_HZ = 20
    STREAM_VFR_HZ      = 10

    # Watchdog: main process'ten bu kadar sn heartbeat gelmezse → RTL
    WATCHDOG_TIMEOUT_SEC = 8.0

# ══════════════════════════════════════════════════════════════════
#  Uçuş Parametreleri
# ══════════════════════════════════════════════════════════════════
class Flight:
    TARGET_ALTITUDE_M   = 15.0    # Kalkış irtifası (m)
    FOLLOW_DISTANCE_M   = 8.0     # Hedef takip mesafesi (m)
    MAX_SPEED_MPS       = 6.5     # Maksimum yatay hız (m/s) — güvenlik sınırı
    MAX_VERTICAL_MPS    = 1.5     # Maksimum dikey hız (m/s)
    MAX_YAW_RATE_DPS    = 40.0    # Maksimum yaw hızı (°/s)
    YAW_SPEED_DPS       = 20.0    # Varsayılan yaw dönüş hızı

    # Hız komutu yenileme (ms) — ArduPilot 3s timeout'tan önce yenilenmeli
    VEL_REFRESH_MS      = 100     # 10 Hz

# ══════════════════════════════════════════════════════════════════
#  PID Kontrolcü
# ══════════════════════════════════════════════════════════════════
class PID:
    # Yatay (YAW) — düşük kp, hafif kd (agresif olmayan)
    YAW_KP   = 0.025;  YAW_KI   = 0.0003;  YAW_KD   = 0.012

    # İleri/Geri (FORWARD — bbox alanına göre mesafe kontrolü)
    FWD_KP   = 0.030;  FWD_KI   = 0.0005;  FWD_KD   = 0.015

    # Dikey (ALTITUDE — piksel hatası)
    ALT_KP   = 0.022;  ALT_KI   = 0.0003;  ALT_KD   = 0.010

    # Çıkış sınırları (m/s)
    MAX_YAW_VEL    = 2.0    # Yaw (vy olarak modelleniyor)
    MAX_FWD_VEL    = 5.0    # İleri/geri
    MAX_ALT_VEL    = 1.2    # Dikey

    # Integral windup koruması
    INTEGRAL_CLAMP = 30.0

    # Eski uyumluluk (vehicle_tracker.py için)
    LATERAL_KP  = 0.025;  LATERAL_KI  = 0.0003;  LATERAL_KD  = 0.012
    VERTICAL_KP = 0.022;  VERTICAL_KI = 0.0003;  VERTICAL_KD = 0.010
    FORWARD_KP  = 0.030;  FORWARD_KI  = 0.0005;  FORWARD_KD  = 0.015
    MAX_LATERAL_VEL  = 2.0
    MAX_VERTICAL_VEL = 1.2
    MAX_FORWARD_VEL  = 5.0

# ══════════════════════════════════════════════════════════════════
#  Kalman Filtre
# ══════════════════════════════════════════════════════════════════
class Kalman:
    # Süreç gürültüsü (küçük = daha az hız değişimine inan)
    PROCESS_NOISE_POS  = 0.01
    PROCESS_NOISE_VEL  = 0.1

    # Ölçüm gürültüsü (piksel bazlı, kamera titremeye göre)
    MEAS_NOISE_POS     = 5.0

    # Gelecek tahmin penceresi (saniye)
    PREDICT_AHEAD_SEC  = 0.20    # 200 ms

    # Maksimum güvenilir hız (piksel/saniye) — aşarsa sıfırla
    MAX_VEL_PX_S       = 800.0

# ══════════════════════════════════════════════════════════════════
#  Takip / Kesme Stratejisi
# ══════════════════════════════════════════════════════════════════
class Pursuit:
    # Bbox alan oranı hedefleri (frame alanının yüzdesi)
    BBOX_AREA_TOO_CLOSE = 0.28   # > bu: geri git
    BBOX_AREA_TARGET    = 0.15   # ideal mesafe (~15%)
    BBOX_AREA_TOO_FAR   = 0.07   # < bu: yaklaş

    # Merkez hata dead-zone (piksel) — bu içinde yaw düzeltmesi yapma
    CENTER_DEADZONE_PX  = 20

    # Hedef kaybı toleransı (kare sayısı)
    MAX_MISS_FRAMES     = 25

    # Kesme açısı (derece) — saf takip yerine çapraz/önden kesme
    # 0 = direkt takip, 15-20 = hafif kesme açısı
    INTERCEPT_ANGLE_DEG = 15.0

    # Kesme stratejisi aktif mi?
    ENABLE_INTERCEPT    = True

    # Latency hedefi (ms) — bu aşılırsa uyarı ver
    TARGET_LATENCY_MS   = 180

# ══════════════════════════════════════════════════════════════════
#  Kamera
# ══════════════════════════════════════════════════════════════════
class Camera:
    INDEX      = 0
    WIDTH      = 640
    HEIGHT     = 480
    FPS        = 30       # Yakalama FPS (PiCamera2)
    FRAME_SKIP = 2        # Her N karede capture → YOLO'ya gönder

# ══════════════════════════════════════════════════════════════════
#  ALPR / YOLO
# ══════════════════════════════════════════════════════════════════
class ALPR:
    MODEL_PATH      = "models/yolov8n_plate.onnx"
    YOLO_CONFIDENCE = 0.55
    OCR_MIN_SCORE   = 0.72

    # YOLO inference boyutu (INT8 optimum)
    INFER_SIZE      = 416

    # YOLO FPS limiti — inference process bu hızda çalışır
    YOLO_MAX_FPS    = 5

    # OCR cooldown — aynı plaka için tekrar OCR yapma süresi (saniye)
    OCR_COOLDOWN_SEC  = 3.0

    # OCR kuyruğu: en fazla bu kadar kare biriktirilir
    OCR_QUEUE_MAXSIZE = 3

    # OCR Languages
    OCR_LANGUAGES   = ["tr", "en"]

    PLATE_PATTERN   = r"^[0-9]{2}[A-Z]{1,3}[0-9]{2,4}$"
    USE_GPU         = False
    FRAME_SKIP      = 3   # eski uyumluluk

# ══════════════════════════════════════════════════════════════════
#  WebSocket / GCS
# ══════════════════════════════════════════════════════════════════
class WebSocket:
    HOST         = "0.0.0.0"
    PORT         = 8765
    STATIC_DIR   = "static"
    TELEMETRY_HZ = 10.0

class GCS:
    IP            = os.getenv("GCS_IP",   "192.168.1.100")
    PORT          = int(os.getenv("GCS_PORT", "14550"))
    SEND_INTERVAL = 2.0

# ══════════════════════════════════════════════════════════════════
#  Fail-Safe
# ══════════════════════════════════════════════════════════════════
class FailSafe:
    BATTERY_WARN_PCT = 30
    BATTERY_RTL_PCT  = 20
    BATTERY_LAND_PCT = 10
    MAX_MISS_FRAMES  = 25
    GPS_MIN_SATS     = 6

# ══════════════════════════════════════════════════════════════════
#  Loglama
# ══════════════════════════════════════════════════════════════════
class Log:
    DIR       = "logs"
    LEVEL     = "INFO"
    ROTATE_MB = 10
