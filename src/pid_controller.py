"""
src/pid_controller.py

Bağımsız, thread-safe PID aks implementasyonu.
Düşük kp, hafif kd — şehir içi takip için agresif olmayan ayarlar.
"""
import time
import threading
import logging

logger = logging.getLogger(__name__)


class PIDAxis:
    """
    Tek eksen PID.

    Anti-windup: integral sınırlama ile.
    Derivative: türevi ölçüm değişimine göre hesapla (set-point jump yok).
    Thread-safe: _lock ile.
    """

    def __init__(
        self,
        kp: float,
        ki: float,
        kd: float,
        output_limit: float  = 10.0,
        integral_clamp: float = 30.0,
        name: str = "PID",
    ):
        self.kp             = kp
        self.ki             = ki
        self.kd             = kd
        self.output_limit   = output_limit
        self.integral_clamp = integral_clamp
        self.name           = name

        self._integral  = 0.0
        self._prev_err  = 0.0
        self._prev_time = time.monotonic()
        self._lock      = threading.Lock()

    # ──────────────────────────────────────────────────────────
    def compute(self, error: float) -> float:
        """
        PID çıkışını hesapla.
        error > 0 → pozitif düzeltme
        """
        with self._lock:
            now = time.monotonic()
            dt  = now - self._prev_time
            dt  = max(0.001, min(dt, 0.5))   # 1ms – 500ms

            # Proportional
            p_term = self.kp * error

            # Integral (windup koruması)
            self._integral += error * dt
            self._integral  = max(
                -self.integral_clamp,
                min(self.integral_clamp, self._integral)
            )
            i_term = self.ki * self._integral

            # Derivative
            d_term = self.kd * (error - self._prev_err) / dt

            output = p_term + i_term + d_term
            # Çıkış limitle
            output = max(-self.output_limit, min(self.output_limit, output))

            self._prev_err  = error
            self._prev_time = now

            logger.debug(
                f"[PID:{self.name}] err={error:+.1f} "
                f"P={p_term:+.3f} I={i_term:+.3f} D={d_term:+.3f} "
                f"→ {output:+.3f}"
            )
            return output

    def reset(self):
        with self._lock:
            self._integral  = 0.0
            self._prev_err  = 0.0
            self._prev_time = time.monotonic()
        logger.debug(f"[PID:{self.name}] Sıfırlandı")

    def set_gains(self, kp: float, ki: float, kd: float):
        with self._lock:
            self.kp = kp
            self.ki = ki
            self.kd = kd


# ════════════════════════════════════════════════════════════════
#  Üç eksen PID grubu (yaw + forward + altitude)
# ════════════════════════════════════════════════════════════════
class TrackingPID:
    """
    Üç eksen PID:
      yaw     → vy (yatay hız — kameraya göre sağ/sol)
      forward → vx (ileri/geri hız — bbox alanına göre)
      alt     → vz (yükseklik hızı — piksel y hatasına göre)
    """

    def __init__(self, cfg=None):
        if cfg is None:
            from config.settings import PID as cfg

        self.yaw = PIDAxis(
            kp=cfg.YAW_KP, ki=cfg.YAW_KI, kd=cfg.YAW_KD,
            output_limit=cfg.MAX_YAW_VEL,
            integral_clamp=cfg.INTEGRAL_CLAMP,
            name="YAW",
        )
        self.forward = PIDAxis(
            kp=cfg.FWD_KP, ki=cfg.FWD_KI, kd=cfg.FWD_KD,
            output_limit=cfg.MAX_FWD_VEL,
            integral_clamp=cfg.INTEGRAL_CLAMP,
            name="FWD",
        )
        self.alt = PIDAxis(
            kp=cfg.ALT_KP, ki=cfg.ALT_KI, kd=cfg.ALT_KD,
            output_limit=cfg.MAX_ALT_VEL,
            integral_clamp=cfg.INTEGRAL_CLAMP,
            name="ALT",
        )

    def reset_all(self):
        self.yaw.reset()
        self.forward.reset()
        self.alt.reset()
