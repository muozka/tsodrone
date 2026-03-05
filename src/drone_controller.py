"""
Drone Kontrol Sistemi — Yuksek Seviye Ucus Kontrolcusu
src/drone_controller.py

Hedef  : Pixhawk 2.4.8 + ArduCopter firmware
Python : 3.13+
"""
import sys, os, time, logging, threading
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from src.mavlink_connection import (
    MAVLinkConnection,
    COPTER_MODE_GUIDED, COPTER_MODE_LOITER, COPTER_MODE_STABILIZE,
)
from config.settings import MAVLink, Flight, FailSafe

logger = logging.getLogger(__name__)


class DroneController:
    """
    Yuksek seviye ucus kontrolu — Pixhawk 2.4.8 / ArduCopter icin optimize.

    Manuel kontrol akisi:
      1. set_mode_guided()   — GUIDED moda gec
      2. send_ned_velocity() — hiz komutu (her 80-200ms yenilenmeli)
      3. Hover icin: send_ned_velocity(0,0,0) veya set_mode_loiter()

    OFFBOARD kavramı ArduPilot'ta yok; bunun karşılığı GUIDED modudur.
    """

    def __init__(self):
        self._conn: MAVLinkConnection = MAVLinkConnection(
            uri  = MAVLink.CONNECTION_STRING,
            baud = MAVLink.BAUD_RATE,
        )
        self.connected      = False
        self._mission_start = None
        # Manuel mod aktif mi? (GCS'den toggle edilir)
        self._manual_active = False

    # BAGLAN
    def connect(self) -> bool:
        ok = self._conn.connect(timeout=MAVLink.TIMEOUT_SEC)
        self.connected = ok
        if ok:
            logger.info(
                f"[DC] Pixhawk hazir | "
                f"GPS={self._conn.gps_sats}sat | "
                f"Pil={self._conn.bat_pct:.0f}% | "
                f"Mod={self._conn.mode}"
            )
        return ok

    # ARM + KALKIS
    def arm_and_takeoff(self, altitude: float = None) -> bool:
        if altitude is None:
            altitude = Flight.TARGET_ALTITUDE_M

        if self._conn.gps_sats < FailSafe.GPS_MIN_SATS:
            logger.warning(
                f"[DC] GPS yetersiz: {self._conn.gps_sats} uydu "
                f"(min {FailSafe.GPS_MIN_SATS})"
            )

        ok = self._conn.takeoff(altitude)
        if ok:
            self._mission_start = time.time()
        return ok

    # NED HIZ KOMUTU — Manuel / Otomatik kontrol icin
    def send_ned_velocity(self, vx: float, vy: float, vz: float,
                          duration: float = 0.1):
        """
        ArduCopter GUIDED modda NED hiz komutu.

        Onemli: ArduPilot GUIDED modda hiz komutunu 3 saniye
        almazsa drone otomatik olarak durur. Bu nedenle GCS
        tarafinda en az her 1 saniyede bir (ideali 80-200ms)
        bu komut yenilenmeli.

        Hiz limitleme:
          yatay: Flight.MAX_SPEED_MPS (varsayilan 8 m/s)
          dikey: 2.0 m/s (guvenlik)
        """
        if not self.connected:
            return

        def clamp(v, mx): return max(-mx, min(mx, v))
        vx = clamp(vx, Flight.MAX_SPEED_MPS)
        vy = clamp(vy, Flight.MAX_SPEED_MPS)
        vz = clamp(vz, 2.0)

        t_end = time.time() + duration
        while time.time() < t_end:
            self._conn.send_ned_velocity(vx, vy, vz)
            time.sleep(0.05)

    def send_ned_velocity_with_yaw(self, vx: float, vy: float, vz: float,
                                    yaw_rate_dps: float):
        """Hiz + yaw hizi kombinasyonu. GUIDED modda calisir."""
        if not self.connected:
            return
        self._conn.send_ned_velocity_with_yaw_rate(vx, vy, vz, yaw_rate_dps)

    def hover(self):
        """
        Hareketsiz kal.
        ArduCopter'da en guvenilir hover: LOITER moduna gec.
        Alternatif: hiz=0 komutu gondermek de calisir ama LOITER daha stabil.
        """
        self._conn.send_ned_velocity(0, 0, 0)
        # 3 paket gonder — ArduPilot timeout'u sifirla
        for _ in range(3):
            self._conn.send_ned_velocity(0, 0, 0)
            time.sleep(0.05)

    # YAW
    def set_yaw(self, heading: float, relative: bool = False):
        self._conn.set_yaw(heading, relative, Flight.YAW_SPEED_DPS)

    # INIS / RTL
    def land(self):
        self._conn.land()

    def return_to_launch(self):
        self._conn.return_to_launch()

    def disarm(self):
        self._conn.disarm()

    # LOITER
    def set_loiter(self):
        """Mevcut konumda sabit kal — ArduCopter LOITER modu."""
        self._conn.set_mode_loiter()

    # GUIDED MOD — Manuel kontrol icin
    def enable_guided(self):
        """
        GUIDED moda gec — manuel kontrol icin kullanilir.
        ArduPilot'ta OFFBOARD modu yoktur; GUIDED karsiligi bu.
        NOT: PX4 OFFBOARD'dan farkli: pre-stream gerekmiyor.
        """
        logger.info("[DC] GUIDED moda geciliyor (manuel kontrol)...")
        return self._conn.set_mode_guided()

    # Eski uyumluluk aliaslari (ws_server.py'den gelen cagrilar)
    def enable_offboard(self):
        """
        Eski PX4 arayuzu uyumlulugu.
        ArduCopter'da OFFBOARD yok — GUIDED kullanilir.
        """
        return self.enable_guided()

    def set_loiter_old(self):
        return self.set_loiter()

    # TELEMETRI
    @property
    def telemetry(self) -> dict:
        snap = self._conn.snapshot
        snap["mission_time"] = (
            int(time.time() - self._mission_start)
            if self._mission_start else 0
        )
        return snap

    # KAPAT
    def disconnect(self):
        self._conn.close()
        self.connected = False
