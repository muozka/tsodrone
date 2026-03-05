"""
Drone Plaka Takip Sistemi - Telemetri Monitoru (Python 3.13 uyumlu)
src/telemetry.py
"""
import threading, logging, time
import sys, os
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from config.settings import FailSafe

logger = logging.getLogger(__name__)


class TelemetryMonitor(threading.Thread):
    """
    Arka planda telemetri loglar.
    Kritik pil seviyelerinde otomatik fail-safe tetikler.

    ONEMLI: bat_ready=True gelene kadar (ilk SYS_STATUS paketi alinana kadar)
    pil fail-safe kontrolleri YAPILMAZ.
    Bu sayede baslangicta bat_pct=0.0 dummy degerinden kaynakli
    yanlis LAND/RTL tetiklenmesi onlenir.
    """

    def __init__(self, drone, interval: float = 2.0):
        super().__init__(daemon=True, name="TelemetryMonitor")
        self.drone    = drone
        self.interval = interval
        self._stop    = threading.Event()
        self._warned  = set()

    def run(self):
        while not self._stop.is_set():
            try:
                t = self.drone.telemetry
                if not t:
                    time.sleep(self.interval)
                    continue

                logger.info(
                    "[TEL] "
                    f"Alt={t.get('alt', 0):.1f}m  "
                    f"Spd={t.get('speed', 0):.1f}m/s  "
                    f"Hdg={t.get('heading', 0):.0f}deg  "
                    f"Bat={t.get('battery', -1):.0f}%/{t.get('voltage', 0):.1f}V  "
                    f"GPS={t.get('satellites', 0)}sat  "
                    f"Mod={t.get('mode', '?')}"
                )

                # ── PIL FAIL-SAFE ────────────────────────────────────
                # bat_ready=False ise gercek pil verisi henuz gelmedi.
                # 0.0 dummy degerine gore islem YAPMA.
                if not t.get("bat_ready", False):
                    logger.debug("Pil verisi bekleniyor (bat_ready=False), fail-safe atlanıyor.")
                    time.sleep(self.interval)
                    continue

                bat = t.get("battery", 100.0)

                if bat <= FailSafe.BATTERY_LAND_PCT and "land" not in self._warned:
                    logger.critical(f"!!! KRITIK PIL ({bat:.0f}%) — OTOMATIK LAND !!!")
                    self._warned.add("land")
                    self.drone.land()

                elif bat <= FailSafe.BATTERY_RTL_PCT and "rtl" not in self._warned:
                    logger.error(f"!! DUSUK PIL ({bat:.0f}%) — OTOMATIK RTL !!")
                    self._warned.add("rtl")
                    self.drone.return_to_launch()

                elif bat <= FailSafe.BATTERY_WARN_PCT and "warn" not in self._warned:
                    logger.warning(f"! PIL UYARISI ({bat:.0f}%)")
                    self._warned.add("warn")

            except Exception as e:
                logger.error(f"Telemetri okuma hatasi: {e}")

            time.sleep(self.interval)

    def stop(self):
        self._stop.set()
