"""
src/pursuit_controller.py

Takip + Kesme Stratejisi — Kalman tahminli, PID kontrollü.

Mimari:
  - Kalman Filtre: bbox merkezini filtreler, 200ms sonraki konumu tahmin eder
  - PID (yaw): tahmin edilen konuma yaw hızı üretir
  - PID (forward): bbox alanına göre ileri/geri hız üretir
  - PID (alt): dikey piksel hatasına göre irtifa hızı üretir
  - Kesme stratejisi: hedef hareket yönüne 15° açıyla kesme açısı ekler
  - Latency monitor: sistem gecikmesini ölçer ve loglar

Kullanım:
    ctrl = PursuitController(command_queue)
    for each detection:
        vx, vy, vz = ctrl.update(bbox, frame_size)
        # command_queue'ya gönderilir
"""
import math
import time
import logging

import numpy as np

from src.kalman_tracker import KalmanTracker2D
from src.pid_controller  import TrackingPID

logger = logging.getLogger(__name__)


class PursuitController:
    """
    Görüntü tabanlı hedef takip + kesme stratejisi.

    command_queue: multiprocessing.Queue — MAVLink process'ine
                   {"cmd": "velocity", "vx": .., "vy": .., "vz": .., "yaw_rate": ..}
                   komutları gönderilir.
    """

    def __init__(self, command_queue, cfg=None):
        if cfg is None:
            from config.settings import Camera, Pursuit, Kalman, PID, Flight
            self._cam   = Camera
            self._pcfg  = Pursuit
            self._kcfg  = Kalman
            self._fcfg  = Flight
        else:
            self._cam, self._pcfg, self._kcfg, self._fcfg = cfg

        self._q = command_queue

        self._kf  = KalmanTracker2D(
            process_noise_pos = self._kcfg.PROCESS_NOISE_POS,
            process_noise_vel = self._kcfg.PROCESS_NOISE_VEL,
            meas_noise_pos    = self._kcfg.MEAS_NOISE_POS,
            max_vel_px_s      = self._kcfg.MAX_VEL_PX_S,
        )
        self._pid = TrackingPID()

        self._miss_count     = 0
        self._last_detect_ts = 0.0
        self._guided_enabled = False

        # Latency ölçümü
        self._latency_sum   = 0.0
        self._latency_count = 0
        self._last_lat_log  = time.monotonic()

        # Frame boyutu (ilk detect'te set edilir)
        self._fw = self._cam.WIDTH
        self._fh = self._cam.HEIGHT
        self._frame_cx = self._fw // 2
        self._frame_cy = self._fh // 2
        self._frame_area = float(self._fw * self._fh)

    # ════════════════════════════════════════════════════════════
    def update(self, bbox: tuple | None,
               frame_w: int = None, frame_h: int = None,
               detect_ts: float = None):
        """
        Her detection döngüsünde çağrılır.
        bbox = (x1, y1, x2, y2) veya None
        detect_ts = tespitın yapıldığı zaman damgası (latency ölçümü için)
        """
        now = time.monotonic()

        # Latency ölçümü
        if detect_ts is not None:
            lat_ms = (now - detect_ts) * 1000.0
            self._latency_sum   += lat_ms
            self._latency_count += 1
            if now - self._last_lat_log > 10.0:
                avg_lat = self._latency_sum / max(1, self._latency_count)
                from config.settings import Pursuit as _P
                tag = "⚠" if avg_lat > _P.TARGET_LATENCY_MS else "✓"
                logger.info(f"[PURSUIT] Ortalama gecikme: {avg_lat:.0f}ms {tag}")
                self._latency_sum   = 0.0
                self._latency_count = 0
                self._last_lat_log  = now

        if frame_w:
            self._fw = frame_w; self._fh = frame_h
            self._frame_cx = frame_w // 2
            self._frame_cy = frame_h // 2
            self._frame_area = float(frame_w * frame_h)

        # ── Hedef kayboldu ──────────────────────────────────────
        if bbox is None:
            self._miss_count += 1
            self._kf.predict_only()

            if self._miss_count >= self._pcfg.MAX_MISS_FRAMES:
                logger.warning("[PURSUIT] Hedef kayboldu — hover")
                self._pid.reset_all()
                self._kf.reset()
                self._miss_count = 0
                self._send_hover()
            return None

        # ── Hedef bulundu ────────────────────────────────────────
        self._miss_count = 0
        self._last_detect_ts = now

        x1, y1, x2, y2 = bbox
        cx = (x1 + x2) / 2.0
        cy = (y1 + y2) / 2.0

        # Kalman güncelle
        fx, fy = self._kf.update(cx, cy)

        # 200ms sonraki tahmini konum
        future = self._kf.predict_future(self._kcfg.PREDICT_AHEAD_SEC)
        if future is not None:
            target_x, target_y = future
        else:
            target_x, target_y = fx, fy

        # Bbox alanı
        bbox_area = float((x2 - x1) * (y2 - y1))
        area_ratio = bbox_area / max(1.0, self._frame_area)

        # ── KESME STRATEJİSİ ─────────────────────────────────────
        # Hedef hareket yönünü Kalman hızından al
        # Kesme açısı: hedef hız yönüne DIKE bir miktar offset
        vx_kf, vy_kf = self._kf.velocity  # px/s

        if self._pcfg.ENABLE_INTERCEPT:
            target_x, target_y = self._apply_intercept(
                target_x, target_y, vx_kf, vy_kf
            )

        # ── PID HESAPLAMA ────────────────────────────────────────

        # YAW: tahmin edilen x konumundan merkez hatası
        ex = target_x - self._frame_cx
        vy = 0.0
        if abs(ex) > self._pcfg.CENTER_DEADZONE_PX:
            vy = self._pid.yaw.compute(ex)

        # FORWARD: bbox alanı hatası
        area_error = self._pcfg.BBOX_AREA_TARGET - area_ratio
        # Negatif hata = çok yakın → geri; Pozitif = uzak → ileri
        if area_ratio > self._pcfg.BBOX_AREA_TOO_CLOSE:
            fwd_error = area_error * 100.0   # geri çekil
        elif area_ratio < self._pcfg.BBOX_AREA_TOO_FAR:
            fwd_error = area_error * 100.0   # yaklaş
        else:
            fwd_error = 0.0  # dead-zone
        vx = self._pid.forward.compute(fwd_error)

        # ALTITUDE: y piksel hatası
        ey = target_y - self._frame_cy
        vz = self._pid.alt.compute(ey)
        # NED: ey > 0 (hedef aşağıda) → yüksel (vz < 0)
        vz = -vz

        # ── HIZ SINIRLAMA ────────────────────────────────────────
        def clamp(v, mx): return max(-mx, min(mx, v))
        vx = clamp(vx, self._fcfg.MAX_SPEED_MPS)
        vy = clamp(vy, self._fcfg.MAX_SPEED_MPS)
        vz = clamp(vz, self._fcfg.MAX_VERTICAL_MPS)

        logger.debug(
            f"[PURSUIT] ex={ex:+.0f}px area={area_ratio:.2f} "
            f"→ vx={vx:+.2f} vy={vy:+.2f} vz={vz:+.2f} m/s"
        )

        # GUIDED mod geçişi (ilk kez)
        if not self._guided_enabled:
            self._send_cmd({"cmd": "set_mode_guided"})
            self._guided_enabled = True

        # Hız komutunu kuyruğa gönder
        self._send_cmd({
            "cmd":      "velocity",
            "vx":       round(vx, 3),
            "vy":       round(vy, 3),
            "vz":       round(vz, 3),
            "yaw_rate": 0.0,
        })

        return vx, vy, vz

    # ════════════════════════════════════════════════════════════
    def _apply_intercept(
        self,
        target_x: float, target_y: float,
        vel_x: float, vel_y: float,
    ) -> tuple[float, float]:
        """
        Kesme stratejisi:
        Hedef sağa doğru hareket ediyorsa, drone hafifçe daha sağa
        bakar — saf takip yerine çapraz kesme açısı.
        Bu şekilde yüksek hız durumlarında hedefi önden kesilebilir.

        Açı: INTERCEPT_ANGLE_DEG (15°)
        Etkinleştirme: yalnızca hedef yeterli hızda hareket ediyorsa.
        """
        speed = math.hypot(vel_x, vel_y)
        MIN_SPEED_PX = 50.0   # bu altında kesme yapmaya gerek yok
        if speed < MIN_SPEED_PX:
            return target_x, target_y

        angle_rad = math.radians(self._pcfg.INTERCEPT_ANGLE_DEG)

        # Hareket yönü açısı
        motion_angle = math.atan2(vel_y, vel_x)

        # Kesme ofset — hareket yönüne dik, ileri kaymış
        # Yani: hedef sağa gidiyorsa, bizim hedef noktamız biraz daha sağda olsun
        intercept_dist = speed * 0.20  # 200ms tahmin mesafesine oranla

        # Offset vektörü: hareket yönünde + açı offset
        off_x = intercept_dist * math.cos(motion_angle + angle_rad)
        off_y = intercept_dist * math.sin(motion_angle + angle_rad)

        # Piksel sınırı içinde tut
        new_x = max(0.0, min(self._fw, target_x + off_x))
        new_y = max(0.0, min(self._fh, target_y + off_y))

        return new_x, new_y

    # ════════════════════════════════════════════════════════════
    def _send_cmd(self, cmd: dict):
        try:
            self._q.put_nowait(cmd)
        except Exception:
            # Kuyruk doluysa eski komutu at
            try:
                self._q.get_nowait()
                self._q.put_nowait(cmd)
            except Exception:
                pass

    def _send_hover(self):
        self._send_cmd({"cmd": "velocity", "vx": 0, "vy": 0, "vz": 0, "yaw_rate": 0})

    def reset(self):
        """Takip tamamen sıfırla."""
        self._kf.reset()
        self._pid.reset_all()
        self._miss_count     = 0
        self._guided_enabled = False
        self._send_hover()
        logger.info("[PURSUIT] Sıfırlandı — hover")
