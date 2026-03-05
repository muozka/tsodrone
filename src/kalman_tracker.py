"""
src/kalman_tracker.py

Kalman Filtre tabanlı 2D konum + hız tahmini.
Gelecek konum tahmini (predict_ahead_sec) ile drone komutları
hedefe değil, 200ms sonraki konuma göre üretilir.

Durum vektörü: [x, y, vx, vy]  (piksel, piksel/s)

Koordinatlar: görüntü piksel uzayı (origin: sol-üst)
"""
import time
import logging
import numpy as np

logger = logging.getLogger(__name__)


class KalmanTracker2D:
    """
    Kalman Filtre 2D tracker.

    Yalnızca görüntü piksel koordinatlarında çalışır.
    Kullanım:
        kf = KalmanTracker2D()
        for each frame:
            if detected:
                predicted_pos = kf.update(cx, cy)
            else:
                predicted_pos = kf.predict_only()
            future_pos = kf.predict_future(0.20)
    """

    def __init__(
        self,
        process_noise_pos: float = 0.01,
        process_noise_vel: float = 0.1,
        meas_noise_pos:    float = 5.0,
        max_vel_px_s:      float = 800.0,
    ):
        # Durum boyutu: 4 (x, y, vx, vy)
        # Ölçüm boyutu: 2 (x, y)
        self._n  = 4
        self._m  = 2

        # Durum geçiş matrisi F (dt ile güncellenir)
        self._F = np.eye(4, dtype=np.float64)

        # Ölçüm matrisi H: sadece pozisyon gözlemleniyor
        self._H = np.array([
            [1, 0, 0, 0],
            [0, 1, 0, 0],
        ], dtype=np.float64)

        # Süreç gürültüsü kovaryansı Q
        self._qp = process_noise_pos
        self._qv = process_noise_vel
        self._Q  = np.diag([self._qp, self._qp, self._qv, self._qv])

        # Ölçüm gürültüsü kovaryansı R
        self._R = np.eye(2, dtype=np.float64) * (meas_noise_pos ** 2)

        # Durum ve kovaryans (başlangıçta bilinmiyor)
        self._x = None    # [x, y, vx, vy]
        self._P = None    # 4x4 kovaryans

        self._max_vel    = max_vel_px_s
        self._last_time  = None
        self._initialized = False

    # ────────────────────────────────────────────────────────────
    def reset(self):
        """Takip sıfırla — hedef kaybolduğunda çağrılır."""
        self._x           = None
        self._P           = None
        self._last_time   = None
        self._initialized = False
        logger.debug("[KF] Sıfırlandı")

    def is_initialized(self) -> bool:
        return self._initialized

    # ────────────────────────────────────────────────────────────
    def update(self, cx: float, cy: float) -> tuple[float, float]:
        """
        Yeni ölçüm (cx, cy) ile güncelle.
        Returns: (filtered_x, filtered_y)
        """
        now = time.monotonic()
        z   = np.array([[cx], [cy]], dtype=np.float64)

        if not self._initialized:
            # İlk ölçüm: durumu direkt başlat
            self._x = np.array([[cx], [cy], [0.0], [0.0]], dtype=np.float64)
            # Yüksek başlangıç belirsizliği
            self._P = np.diag([100.0, 100.0, 1000.0, 1000.0])
            self._last_time   = now
            self._initialized = True
            return cx, cy

        # dt hesapla
        dt = now - self._last_time
        dt = max(1e-4, min(dt, 0.5))   # 0.1ms – 500ms sınırı
        self._last_time = now

        # F matrisini dt ile güncelle
        self._F[0, 2] = dt
        self._F[1, 3] = dt

        # Q matrisini dt'ye göre ölçekle
        Q = self._Q.copy()
        Q[0, 0] *= dt ** 2
        Q[1, 1] *= dt ** 2
        Q[2, 2] *= dt
        Q[3, 3] *= dt

        # ── Tahmin adımı ──
        x_pred = self._F @ self._x
        P_pred = self._F @ self._P @ self._F.T + Q

        # ── Güncelleme adımı ──
        y     = z - self._H @ x_pred           # inovasyon
        S     = self._H @ P_pred @ self._H.T + self._R
        K     = P_pred @ self._H.T @ np.linalg.inv(S)
        self._x = x_pred + K @ y
        I = np.eye(self._n)
        self._P = (I - K @ self._H) @ P_pred

        # Hız sınırla (unrealistic jump'ları engelle)
        self._x[2, 0] = np.clip(self._x[2, 0], -self._max_vel, self._max_vel)
        self._x[3, 0] = np.clip(self._x[3, 0], -self._max_vel, self._max_vel)

        return float(self._x[0, 0]), float(self._x[1, 0])

    # ────────────────────────────────────────────────────────────
    def predict_only(self) -> tuple[float, float] | None:
        """
        Ölçüm olmadan yalnızca tahmin adımı.
        Kaçırılan karelerde belirsizliği artırır.
        """
        if not self._initialized:
            return None

        now = time.monotonic()
        dt  = max(1e-4, min(now - self._last_time, 0.5))
        self._last_time = now

        self._F[0, 2] = dt
        self._F[1, 3] = dt
        self._x = self._F @ self._x
        Q = self._Q.copy()
        Q[0, 0] *= dt ** 2; Q[1, 1] *= dt ** 2
        Q[2, 2] *= dt;      Q[3, 3] *= dt
        self._P = self._F @ self._P @ self._F.T + Q * 3.0  # Belirsizlik artar

        return float(self._x[0, 0]), float(self._x[1, 0])

    # ────────────────────────────────────────────────────────────
    def predict_future(self, ahead_sec: float = 0.20) -> tuple[float, float] | None:
        """
        Mevcut durum ve hızdan ahead_sec saniye sonraki
        tahmini konumu döndür.

        Bu değer drone komutunda hedef olarak kullanılır —
        drone hedefe değil, hedefin 200ms sonraki konumuna uçar.
        """
        if not self._initialized or self._x is None:
            return None

        x_future = float(self._x[0, 0]) + float(self._x[2, 0]) * ahead_sec
        y_future = float(self._x[1, 0]) + float(self._x[3, 0]) * ahead_sec

        return x_future, y_future

    # ────────────────────────────────────────────────────────────
    @property
    def velocity(self) -> tuple[float, float]:
        """Mevcut hız tahmini (px/s)."""
        if not self._initialized:
            return 0.0, 0.0
        return float(self._x[2, 0]), float(self._x[3, 0])

    @property
    def position(self) -> tuple[float, float] | None:
        """Mevcut filtrelenmiş konum."""
        if not self._initialized:
            return None
        return float(self._x[0, 0]), float(self._x[1, 0])

    @property
    def uncertainty(self) -> float:
        """Konum belirsizliği (kovaryans izi — büyüdükçe güvensiz)."""
        if self._P is None:
            return 999.0
        return float(np.trace(self._P[:2, :2]))
