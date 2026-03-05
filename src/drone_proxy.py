"""
src/drone_proxy.py

DroneProxy — ws_server'ı multi-process mimarisinde kullanabilmek için
gerçek DroneController gibi davranan hafif proxy sınıf.

ws_server'daki tüm _real_drone.xxx çağrıları bu sınıfa yönlendirilir.
Asıl komutlar command_queue üzerinden MAVLink process'ine iletilir.
Telemetri ise shared telemetry_state dict'ten okunur.

Bu sayede ws_server kodu değiştirilmeden multi-process mimarisiyle çalışır.
"""
import time
import logging
import threading

logger = logging.getLogger(__name__)


class _ConnProxy:
    """
    _real_drone._conn.xxx çağrıları için iç proxy.
    ws_server'da doğrudan _conn erişimleri var.
    """

    def __init__(self, cmd_queue, telem_state: dict):
        self._q     = cmd_queue
        self._telem = telem_state

    def _send(self, cmd: dict):
        try:
            self._q.put_nowait(cmd)
        except Exception:
            try:
                self._q.get_nowait()
                self._q.put_nowait(cmd)
            except Exception:
                pass

    # Mod geçişleri
    def set_mode_guided(self):    self._send({"cmd": "set_mode_guided"})
    def set_mode_loiter(self):    self._send({"cmd": "set_mode_loiter"})
    def set_mode_stabilize(self): self._send({"cmd": "set_mode_stabilize"})
    def set_mode_rtl(self):       self._send({"cmd": "set_mode_rtl"})
    def set_mode_land(self):      self._send({"cmd": "set_mode_land"})

    # ARM / DISARM
    def arm(self, force: bool = False):
        self._send({"cmd": "arm", "force": force})

    # Hız komutları
    def send_ned_velocity(self, vx, vy, vz):
        self._send({"cmd": "velocity", "vx": vx, "vy": vy, "vz": vz, "yaw_rate": 0})

    def send_ned_velocity_with_yaw_rate(self, vx, vy, vz, yaw_rate):
        self._send({"cmd": "velocity", "vx": vx, "vy": vy, "vz": vz, "yaw_rate": yaw_rate})

    # Motor test
    def motor_test(self, motor_id=0, throttle_pct=10.0, duration_sec=3.0):
        self._send({
            "cmd":      "motor_test",
            "motor_id": motor_id,
            "throttle": throttle_pct,
            "duration": duration_sec,
        })

    # RC Override
    def rc_override_throttle(self, pwm: int = 1000):
        self._send({"cmd": "rc_throttle", "pwm": pwm})

    def rc_override_release(self):
        self._send({"cmd": "rc_release"})

    # Telemetri özellikleri (ws_server okur)
    @property
    def armed(self) -> bool:
        return bool(self._telem.get("armed", False))

    @property
    def mode(self) -> str:
        return self._telem.get("mode", "UNKNOWN")

    @property
    def mode_num(self) -> int:
        return int(self._telem.get("mode_num", -1))


class DroneProxy:
    """
    ws_server için DroneController proxy'si.

    connected: her zaman True (MAVLink process bağlandıysa)
    _conn: _ConnProxy instance'ı (ws_server'ın doğrudan _conn erişimi için)
    """

    def __init__(self, command_queue, telemetry_state: dict):
        """
        command_queue : multiprocessing.Queue — MAVLink process'e komutlar
        telemetry_state: threading.local paylaşımlı dict
                         (main process telemetri queue'dan okuyup günceller)
        """
        self._q     = command_queue
        self._telem = telemetry_state
        self._conn  = _ConnProxy(command_queue, telemetry_state)
        self.connected = True
        self._mission_start = None

    def _send(self, cmd: dict):
        try:
            self._q.put_nowait(cmd)
        except Exception:
            try:
                self._q.get_nowait()
                self._q.put_nowait(cmd)
            except Exception:
                pass

    # DroneController public API
    def arm_and_takeoff(self, altitude: float = 15.0):
        self._send({"cmd": "takeoff", "altitude": altitude})
        self._mission_start = time.time()

    def disarm(self):
        self._send({"cmd": "disarm"})

    def land(self):
        self._send({"cmd": "land"})

    def return_to_launch(self):
        self._send({"cmd": "rtl"})

    def hover(self):
        self._send({"cmd": "velocity", "vx": 0, "vy": 0, "vz": 0, "yaw_rate": 0})

    def send_ned_velocity(self, vx, vy, vz, duration=0.1):
        self._send({"cmd": "velocity", "vx": vx, "vy": vy, "vz": vz, "yaw_rate": 0})

    def set_yaw(self, heading: float, relative: bool = False):
        self._send({"cmd": "yaw", "heading": heading, "relative": relative})

    def set_loiter(self):
        self._send({"cmd": "set_mode_loiter"})

    def enable_offboard(self):
        self._send({"cmd": "set_mode_guided"})
        return True

    def enable_guided(self):
        return self.enable_offboard()

    def disconnect(self):
        pass   # MAVLink process kendi kendini kapatır

    @property
    def telemetry(self) -> dict:
        snap = dict(self._telem)
        snap["mission_time"] = (
            int(time.time() - self._mission_start)
            if self._mission_start else 0
        )
        return snap
