"""
src/vehicle_tracker.py

Geriye dönük uyumluluk sarmalayıcı.
Eski API: VehicleTracker(drone) → update(bbox)

Yeni davranış: PursuitController ve KalmanTracker2D kullanır.
Tek-process modda (demo) command_queue yerine doğrudan drone çağrısı yapar.
"""
import logging
import time
import queue
import threading

logger = logging.getLogger(__name__)


class _LocalQueue:
    """
    Tek-process modda command_queue yerine kullanılan
    hafif in-process kuyruk.
    """
    def __init__(self, drone):
        self._drone = drone
        self._q     = queue.Queue(maxsize=50)
        self._t     = threading.Thread(target=self._worker, daemon=True)
        self._t.start()

    def _worker(self):
        from config.settings import MAVLink as M
        from src.mavlink_connection import COPTER_MODE_GUIDED
        while True:
            try:
                cmd = self._q.get(timeout=0.5)
                action = cmd.get("cmd", "")
                if action == "velocity":
                    vx = cmd.get("vx", 0)
                    vy = cmd.get("vy", 0)
                    vz = cmd.get("vz", 0)
                    yr = cmd.get("yaw_rate", 0)
                    if self._drone.connected:
                        if yr != 0:
                            self._drone._conn.send_ned_velocity_with_yaw_rate(vx, vy, vz, yr)
                        else:
                            self._drone._conn.send_ned_velocity(vx, vy, vz)
                elif action == "set_mode_guided" and self._drone.connected:
                    self._drone._conn.set_mode_guided()
                elif action == "set_mode_loiter" and self._drone.connected:
                    self._drone._conn.set_mode_loiter()
            except queue.Empty:
                pass
            except Exception as e:
                logger.debug(f"[VT] Worker hatası: {e}")

    def put_nowait(self, cmd):
        try:
            self._q.put_nowait(cmd)
        except queue.Full:
            try:
                self._q.get_nowait()
                self._q.put_nowait(cmd)
            except Exception:
                pass

    def put(self, cmd):
        self.put_nowait(cmd)


class VehicleTracker:
    """
    Kalman Filter + PID tabanlı araç takip kontrolcüsü.

    Tek-process uyumluluk modu: drone nesnesi üzerinden çalışır.
    Multi-process modda: PursuitController doğrudan kullanılır.
    """

    def __init__(self, drone):
        self.drone = drone
        self._lq   = _LocalQueue(drone)

        from src.pursuit_controller import PursuitController
        self._pursuit = PursuitController(self._lq)
        logger.info("[VT] Kalman+PID takip kontrolcüsü hazır")

    def update(self, bbox: tuple | None):
        """
        Her kamera karesinde çağrılır.
        bbox = (x1, y1, x2, y2) veya None
        """
        self._pursuit.update(bbox, detect_ts=time.monotonic())

    def reset_offboard(self):
        self._pursuit.reset()
        if self.drone.connected:
            self.drone.set_loiter()
