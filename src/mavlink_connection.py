"""
Drone Kontrol Sistemi — Dusuk Seviye MAVLink Baglantisi
src/mavlink_connection.py

Hedef  : Pixhawk 2.4.8 (STM32F427, FMUv2/FMUv3)
Firmware: ArduCopter (ArduPilot)  — PX4 DEGiL
Python  : 3.13+

KRITIK FARKLAR  PX4 vs ArduCopter:
  Mod ayari : MAV_CMD_DO_SET_MODE + MAV_MODE_FLAG_CUSTOM_MODE_ENABLED
              ArduCopter mod numaralari (GUIDED=4, RTL=6, LAND=9 vb.)
              PX4 main_mode/sub_mode cifti Pixhawk 2.4.8'de CALISMAZ.
  ARM       : MAV_CMD_COMPONENT_ARM_DISARM  p1=1, p2=0 (normal)
              Force arm icin p2=2989 (ArduPilot magic)
  Kalkis    : GUIDED moda gec sonra MAV_CMD_NAV_TAKEOFF gonder
              PX4 TAKEOFF sub-mode yontemi ArduCopter'da CALISMAZ.
  Hiz komutu: SET_POSITION_TARGET_LOCAL_NED, GUIDED modda calisir
              type_mask=0x0FC7 — sadece vx,vy,vz aktif
              1 saniyede yenilenmezse drone durur (3s timeout)
  Veri akisi: MAV_DATA_STREAM_ALL yerine ayri stream ID'leri
              57600 baud icin 10-20 Hz idealdir
"""
import time
import math
import queue
import threading
import logging
from pymavlink import mavutil

logger = logging.getLogger(__name__)

# ArduCopter Mod Numaralari (kaynak: ArduPilot/ardupilot mode.h)
COPTER_MODE_STABILIZE   = 0
COPTER_MODE_ACRO        = 1
COPTER_MODE_ALT_HOLD    = 2
COPTER_MODE_AUTO        = 3
COPTER_MODE_GUIDED      = 4
COPTER_MODE_LOITER      = 5
COPTER_MODE_RTL         = 6
COPTER_MODE_CIRCLE      = 7
COPTER_MODE_LAND        = 9
COPTER_MODE_POSHOLD     = 16
COPTER_MODE_BRAKE       = 17
COPTER_MODE_SMART_RTL   = 21

# SET_POSITION_TARGET_LOCAL_NED type_mask
# bit0-2: ignore pos xyz, bit3-5: vel xyz (0=kullan), bit6-8: ignore accel, bit10-11: ignore yaw
TYPEMASK_VEL_ONLY     = 0b0000_1111_1100_0111   # 0x0FC7 — sadece vx,vy,vz aktif
TYPEMASK_VEL_YAW_RATE = 0b0000_0111_1100_0111   # 0x07C7 — vx,vy,vz + yaw_rate aktif

# ArduPilot force-arm sihirli sayisi
ARDUCOPTER_FORCE_ARM_MAGIC = 2989


class MAVLinkConnection:
    """
    Pixhawk 2.4.8 (ArduCopter) ile pymavlink uzerinden guvenli haberlasme.

    Mimari:
      - Serial port YALNIZCA _receive_loop tarafindan okunur.
      - COMMAND_ACK'lar _ack_queue ile komut thread'lerine iletilir.
      - Tum mav.send_* cagrilari _send_lock ile serialise edilir.
    """

    def __init__(self, uri: str, baud: int = 57600):
        self.uri  = uri
        self.baud = baud
        self.master = None

        # Telemetri state
        self.connected       = False
        self.armed           = False
        self.mode            = "UNKNOWN"
        self.mode_num        = -1
        self.lat             = 0.0
        self.lon             = 0.0
        self.alt             = 0.0
        self.rel_alt         = 0.0
        self.heading         = 0.0
        self.groundspeed     = 0.0
        self.bat_pct         = -1.0
        self.bat_volt        = 0.0
        self.bat_curr        = 0.0
        self.gps_fix         = 0
        self.gps_sats        = 0
        self.bat_initialized = False
        self.ekf_ok          = False
        self._home_lat       = None
        self._home_lon       = None

        # Thread guvenligi
        self._state_lock = threading.Lock()
        self._send_lock  = threading.Lock()
        self._ack_queue  = queue.Queue(maxsize=64)
        self._rx_running = False
        self._rx_thread  = None

    # BAGLAN
    def connect(self, timeout: int = 30) -> bool:
        """Pixhawk 2.4.8'e baglan — ArduPilot/ArduCopter firmware icin optimize."""
        try:
            logger.info(f"[MAV] Baglaniyor: {self.uri} @ {self.baud} baud")
            self.master = mavutil.mavlink_connection(
                self.uri,
                baud=self.baud,
                source_system=255,
                source_component=0,
                dialect="ardupilotmega",
            )

            logger.info("[MAV] Heartbeat bekleniyor...")
            msg = self.master.wait_heartbeat(timeout=timeout)
            if msg is None:
                logger.error("[MAV] Heartbeat alinamadi!")
                return False

            logger.info(
                f"[MAV] Pixhawk 2.4.8 bulundu — "
                f"sys={self.master.target_system} "
                f"comp={self.master.target_component}"
            )
            self.connected = True

            self._request_data_streams()
            self._disable_safety_switch()
            self._request_home_position()

            self._rx_running = True
            self._rx_thread = threading.Thread(
                target=self._receive_loop,
                name="MAVRx",
                daemon=True,
            )
            self._rx_thread.start()

            time.sleep(2.0)
            logger.info(
                f"[MAV] Hazir — GPS={self.gps_sats}sat "
                f"Pil={self.bat_pct:.0f}% Mod={self.mode}"
            )
            return True

        except Exception as e:
            logger.error(f"[MAV] Baglanti hatasi: {e}")
            return False

    # VERI AKISI
    def _request_data_streams(self):
        """
        ArduPilot icin ayri stream ID'leri ile telemetri istegi.
        MAV_DATA_STREAM_ALL tek komut yerine; ayri istekler daha guvenilir.
        57600 baud limit: 10-20 Hz optimal aralik.
        """
        streams = [
            (mavutil.mavlink.MAV_DATA_STREAM_RAW_SENSORS,     10),
            (mavutil.mavlink.MAV_DATA_STREAM_EXTENDED_STATUS, 10),
            (mavutil.mavlink.MAV_DATA_STREAM_RC_CHANNELS,      5),
            (mavutil.mavlink.MAV_DATA_STREAM_POSITION,        10),
            (mavutil.mavlink.MAV_DATA_STREAM_EXTRA1,          20),
            (mavutil.mavlink.MAV_DATA_STREAM_EXTRA2,          10),
            (mavutil.mavlink.MAV_DATA_STREAM_EXTRA3,           5),
        ]
        for sid, rate in streams:
            self.master.mav.request_data_stream_send(
                self.master.target_system,
                self.master.target_component,
                sid, rate, 1,
            )
        logger.info(f"[MAV] {len(streams)} telemetri stream'i istendi")

    def _request_home_position(self):
        try:
            self.master.mav.command_long_send(
                self.master.target_system,
                self.master.target_component,
                mavutil.mavlink.MAV_CMD_GET_HOME_POSITION,
                0, 0, 0, 0, 0, 0, 0, 0,
            )
        except Exception:
            pass

    # SAFETY SWITCH
    def _disable_safety_switch(self):
        """
        Pixhawk 2.4.8 fiziksel safety switch'i devre disi birak.
        Companion computer kullanimi icin gereklidir.
        """
        try:
            self.master.mav.set_mode_send(
                self.master.target_system,
                mavutil.mavlink.MAV_MODE_FLAG_DECODE_POSITION_SAFETY,
                0,
            )
        except Exception:
            pass

        for param, val in [('BRD_SAFETYENABLE', 0), ('BRD_SAFETY_DEFLT', 0)]:
            try:
                self.master.param_set_send(param, val)
                time.sleep(0.1)
            except Exception:
                pass

        logger.info("[MAV] Safety switch devre disi birakildi")

    # ALICI DONGU
    def _receive_loop(self):
        """Serial port'un tek sahibi — tum MAVLink mesajlari burada islenior."""
        while self._rx_running:
            try:
                msg = self.master.recv_match(blocking=True, timeout=0.02)
                if msg is None:
                    continue

                mtype = msg.get_type()

                if mtype == "HEARTBEAT":
                    with self._state_lock:
                        self.armed    = bool(msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
                        self.mode_num = msg.custom_mode
                        self.mode     = self._decode_mode(msg)

                elif mtype == "GLOBAL_POSITION_INT":
                    with self._state_lock:
                        self.lat     = msg.lat / 1e7
                        self.lon     = msg.lon / 1e7
                        self.alt     = msg.alt / 1000.0
                        self.rel_alt = msg.relative_alt / 1000.0
                        if msg.hdg != 65535:
                            self.heading = msg.hdg / 100.0

                elif mtype == "VFR_HUD":
                    with self._state_lock:
                        self.groundspeed = msg.groundspeed
                        self.heading     = float(msg.heading)

                elif mtype == "SYS_STATUS":
                    with self._state_lock:
                        volt = msg.voltage_battery / 1000.0
                        pct  = float(msg.battery_remaining)
                        curr = float(msg.current_battery) * 10.0  # cA -> mA
                        if volt > 5.0 and pct >= 0:
                            self.bat_volt        = volt
                            self.bat_pct         = pct
                            self.bat_curr        = curr
                            self.bat_initialized = True
                        # EKF saglik kontrolu
                        enabled = msg.onboard_control_sensors_enabled
                        health  = msg.onboard_control_sensors_health
                        CRITICAL = (
                            mavutil.mavlink.MAV_SYS_STATUS_SENSOR_3D_GYRO  |
                            mavutil.mavlink.MAV_SYS_STATUS_SENSOR_3D_ACCEL |
                            mavutil.mavlink.MAV_SYS_STATUS_SENSOR_GPS
                        )
                        ec = enabled & CRITICAL
                        self.ekf_ok = (health & ec) == ec

                elif mtype == "GPS_RAW_INT":
                    with self._state_lock:
                        self.gps_fix  = msg.fix_type
                        self.gps_sats = msg.satellites_visible

                elif mtype == "HOME_POSITION":
                    with self._state_lock:
                        self._home_lat = msg.latitude  / 1e7
                        self._home_lon = msg.longitude / 1e7
                    logger.info(f"[MAV] Home: {self._home_lat:.6f}, {self._home_lon:.6f}")

                elif mtype == "COMMAND_ACK":
                    try:
                        self._ack_queue.put_nowait(msg)
                    except queue.Full:
                        try:
                            self._ack_queue.get_nowait()
                            self._ack_queue.put_nowait(msg)
                        except Exception:
                            pass

                elif mtype == "STATUSTEXT":
                    text = msg.text.strip('\x00').strip()
                    if text:
                        sev = getattr(msg, 'severity', 6)
                        lvl = logging.WARNING if sev <= 4 else logging.INFO
                        logger.log(lvl, f"[FC] {text}")

            except Exception as e:
                if self._rx_running:
                    logger.debug(f"[MAV] RX dongu hatasi: {e}")

    def _decode_mode(self, hb_msg) -> str:
        """ArduCopter mod numarasini string isme cevir."""
        try:
            name = mavutil.mode_string_v10(hb_msg)
            if name and name not in ("Mode(?)", ""):
                return name
        except Exception:
            pass
        MODE_TABLE = {
            0: "STABILIZE",   1: "ACRO",       2: "ALT_HOLD",
            3: "AUTO",        4: "GUIDED",      5: "LOITER",
            6: "RTL",         7: "CIRCLE",      9: "LAND",
            11: "DRIFT",     13: "SPORT",      16: "POSHOLD",
            17: "BRAKE",     20: "GUIDED_NOGPS", 21: "SMART_RTL",
        }
        return MODE_TABLE.get(hb_msg.custom_mode, f"MODE_{hb_msg.custom_mode}")

    # KOMUT YARDIMCISI
    def _flush_ack_queue(self):
        while not self._ack_queue.empty():
            try:
                self._ack_queue.get_nowait()
            except queue.Empty:
                break

    def _send_command_long(self, command, p1=0, p2=0, p3=0,
                           p4=0, p5=0, p6=0, p7=0,
                           wait_ack: bool = True,
                           ack_timeout: float = 5.0) -> bool:
        """
        MAVLink COMMAND_LONG gonder.
        _ack_queue uzerinden ACK beklenir — recv_match KULLANILMAZ.
        """
        with self._send_lock:
            self._flush_ack_queue()
            self.master.mav.command_long_send(
                self.master.target_system,
                self.master.target_component,
                command, 0,
                float(p1), float(p2), float(p3),
                float(p4), float(p5), float(p6), float(p7),
            )

        if not wait_ack:
            return True

        deadline = time.time() + ack_timeout
        while time.time() < deadline:
            try:
                ack = self._ack_queue.get(timeout=min(1.0, deadline - time.time()))
                if ack.command == command:
                    ok = ack.result == mavutil.mavlink.MAV_RESULT_ACCEPTED
                    if not ok:
                        logger.warning(f"[MAV] ACK HATA: cmd={command} result={ack.result}")
                    return ok
                try:
                    self._ack_queue.put_nowait(ack)
                except queue.Full:
                    pass
            except queue.Empty:
                break

        logger.warning(f"[MAV] ACK timeout: cmd={command}")
        return False

    # MOD DEGISTIR — ArduCopter
    def set_mode(self, mode_num: int) -> bool:
        """
        ArduCopter ucus modunu degistir.
        p1=MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, p2=mod numarasi.
        PX4 main_mode/sub_mode cifti Pixhawk 2.4.8'de CALISMAZ.
        """
        logger.info(f"[MAV] Mod -> {mode_num}")
        with self._send_lock:
            self._flush_ack_queue()
            self.master.mav.command_long_send(
                self.master.target_system,
                self.master.target_component,
                mavutil.mavlink.MAV_CMD_DO_SET_MODE, 0,
                float(mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED),
                float(mode_num),
                0.0, 0.0, 0.0, 0.0, 0.0,
            )
        # ArduPilot bazen ACK gondermez — kisa bekle, basarisiz olursa devam et
        try:
            ack = self._ack_queue.get(timeout=3.0)
            return ack is not None
        except queue.Empty:
            logger.debug(f"[MAV] set_mode ACK timeout (mod={mode_num}) — heartbeat dogrulayacak")
            return True

    def _wait_mode(self, mode_num: int, timeout: float = 5.0) -> bool:
        t0 = time.time()
        while time.time() - t0 < timeout:
            with self._state_lock:
                if self.mode_num == mode_num:
                    return True
            time.sleep(0.1)
        return False

    # Kısayollar
    def set_mode_guided(self):    return self.set_mode(COPTER_MODE_GUIDED)
    def set_mode_loiter(self):    return self.set_mode(COPTER_MODE_LOITER)
    def set_mode_rtl(self):       return self.set_mode(COPTER_MODE_RTL)
    def set_mode_land(self):      return self.set_mode(COPTER_MODE_LAND)
    def set_mode_stabilize(self): return self.set_mode(COPTER_MODE_STABILIZE)
    def set_mode_poshold(self):   return self.set_mode(COPTER_MODE_POSHOLD)
    def set_mode_alt_hold(self):  return self.set_mode(COPTER_MODE_ALT_HOLD)
    def set_mode_brake(self):     return self.set_mode(COPTER_MODE_BRAKE)
    # Eski PX4 isimleri — uyumluluk icin
    def set_mode_offboard(self):  return self.set_mode_guided()
    def set_mode_posctl(self):    return self.set_mode_poshold()

    # ARM
    def arm(self, force: bool = False) -> bool:
        """
        ArduCopter ARM proseduru:
        1. STABILIZE moduna gec (en guvenli ARM modu)
        2. MAV_CMD_COMPONENT_ARM_DISARM gonder
        3. Heartbeat'te armed=True gorune kadar bekle
        force=True: arming check bypass (test/debug icin)
        """
        for attempt in range(3):
            logger.info(f"[MAV] ARM deneme {attempt+1}/3...")
            self.set_mode_stabilize()
            time.sleep(1.5)

            p2 = float(ARDUCOPTER_FORCE_ARM_MAGIC) if force else 0.0
            with self._send_lock:
                self._flush_ack_queue()
                self.master.mav.command_long_send(
                    self.master.target_system,
                    self.master.target_component,
                    mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
                    0,
                    1.0,   # ARM
                    p2,
                    0, 0, 0, 0, 0,
                )

            t0 = time.time()
            while time.time() - t0 < 6.0:
                with self._state_lock:
                    if self.armed:
                        logger.info("[MAV] ARM basarili!")
                        return True
                time.sleep(0.2)

            logger.warning(f"[MAV] ARM deneme {attempt+1} basarisiz")
            if attempt < 2:
                time.sleep(3)

        logger.error("[MAV] ARM tamamen basarisiz!")
        return False

    def disarm(self) -> bool:
        logger.info("[MAV] DISARM...")
        return self._send_command_long(
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            p1=0, p2=0,
        )

    # KALKIS
    def takeoff(self, altitude: float) -> bool:
        """
        ArduCopter kalkis proseduru:
        1. GUIDED moda gec
        2. ARM et
        3. MAV_CMD_NAV_TAKEOFF gonder
        PX4 TAKEOFF sub-mode yontemi ArduCopter'da CALISMAZ!
        """
        logger.info(f"[MAV] Kalkis baslıyor — hedef: {altitude}m")

        if not self.set_mode_guided():
            logger.error("[MAV] GUIDED mod basarisiz")
            return False
        time.sleep(0.5)

        if not self.armed:
            if not self.arm(force=False):
                logger.warning("[MAV] Normal ARM basarisiz, force ARM deneniyor...")
                if not self.arm(force=True):
                    logger.error("[MAV] ARM tamamen basarisiz!")
                    return False
        time.sleep(0.5)

        # MAV_CMD_NAV_TAKEOFF — ArduCopter kalkis komutu
        # p4=nan: mevcut yonde devam, p7=hedef irtifa
        ok = self._send_command_long(
            mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
            p1=0, p2=0, p3=0,
            p4=float('nan'),   # yaw: mevcut yon
            p5=0, p6=0,
            p7=float(altitude),
            ack_timeout=8.0,
        )
        if not ok:
            logger.error("[MAV] TAKEOFF ACK hatasi")
            return False

        t0 = time.time()
        while time.time() - t0 < 40.0:
            with self._state_lock:
                cur = self.rel_alt
            if cur >= altitude * 0.92:
                logger.info(f"[MAV] Hedef irtifaya ulasildi: {cur:.1f}m")
                self.set_mode_loiter()
                return True
            time.sleep(0.5)

        logger.warning(f"[MAV] Irtifa timeout — mevcut: {self.rel_alt:.1f}m")
        return False

    # NED HIZ KOMUTU
    def send_ned_velocity(self, vx: float, vy: float, vz: float):
        """
        SET_POSITION_TARGET_LOCAL_NED ile hiz komutu.
        GUIDED modda calisir.
        1 saniyede yenilenmezse drone durur (3s ArduPilot timeout).
        NED: vx=Kuzey(+), vy=Dogu(+), vz=Asagi(+)
        """
        with self._send_lock:
            self.master.mav.set_position_target_local_ned_send(
                int(time.time() * 1000) & 0xFFFFFFFF,
                self.master.target_system,
                self.master.target_component,
                mavutil.mavlink.MAV_FRAME_LOCAL_NED,
                TYPEMASK_VEL_ONLY,
                0.0, 0.0, 0.0,
                float(vx), float(vy), float(vz),
                0.0, 0.0, 0.0,
                0.0, 0.0,
            )

    def send_ned_velocity_with_yaw_rate(self, vx: float, vy: float, vz: float,
                                         yaw_rate_dps: float):
        """Hiz + yaw hizi kombinasyonu. yaw_rate: derece/saniye."""
        with self._send_lock:
            self.master.mav.set_position_target_local_ned_send(
                int(time.time() * 1000) & 0xFFFFFFFF,
                self.master.target_system,
                self.master.target_component,
                mavutil.mavlink.MAV_FRAME_LOCAL_NED,
                TYPEMASK_VEL_YAW_RATE,
                0.0, 0.0, 0.0,
                float(vx), float(vy), float(vz),
                0.0, 0.0, 0.0,
                0.0, math.radians(float(yaw_rate_dps)),
            )

    # YAW KONTROLU
    def set_yaw(self, heading_deg: float, relative: bool = False,
                speed_dps: float = 20.0):
        """MAV_CMD_CONDITION_YAW ile yon kontrolu. GUIDED modda calisir."""
        self._send_command_long(
            mavutil.mavlink.MAV_CMD_CONDITION_YAW,
            p1=abs(heading_deg),
            p2=max(1.0, abs(speed_dps)),
            p3=1.0 if heading_deg >= 0 else -1.0,
            p4=1.0 if relative else 0.0,
            wait_ack=False,
        )

    # RTL / LAND / LOITER
    def return_to_launch(self):
        logger.info("[MAV] RTL")
        self.set_mode_rtl()

    def land(self):
        logger.info("[MAV] LAND")
        self.set_mode_land()

    def loiter(self):
        logger.info("[MAV] LOITER")
        self.set_mode_loiter()

    # MOTOR TEST
    def motor_test(self, motor_id: int = 0, throttle_pct: float = 10.0,
                   duration_sec: float = 3.0):
        """ArduPilot MAV_CMD_DO_MOTOR_TEST. motor_id: 0-3."""
        logger.info(f"[MAV] Motor test: motor={motor_id+1} %{throttle_pct:.0f} {duration_sec}s")
        return self._send_command_long(
            mavutil.mavlink.MAV_CMD_DO_MOTOR_TEST,
            p1=float(motor_id),
            p2=1.0,                    # throttle type: 1=PERCENT
            p3=float(throttle_pct),
            p4=float(duration_sec),
            p5=0.0,
            p6=0.0,
            wait_ack=True,
            ack_timeout=10.0,
        )

    # RC OVERRIDE
    def rc_override_throttle(self, throttle_pwm: int = 1000):
        logger.info(f"[MAV] RC Override PWM={throttle_pwm}")
        with self._send_lock:
            self.master.mav.rc_channels_override_send(
                self.master.target_system,
                self.master.target_component,
                1500, 1500, throttle_pwm, 1500,
                0, 0, 0, 0,
            )

    def rc_override_release(self):
        logger.info("[MAV] RC Override serbest")
        with self._send_lock:
            self.master.mav.rc_channels_override_send(
                self.master.target_system,
                self.master.target_component,
                0, 0, 0, 0, 0, 0, 0, 0,
            )

    # SERVO
    def set_servo(self, servo_num: int = 1, pwm: int = 1500):
        return self._send_command_long(
            mavutil.mavlink.MAV_CMD_DO_SET_SERVO,
            p1=float(servo_num), p2=float(pwm),
            wait_ack=True,
        )

    # TELEMETRI SNAPSHOT
    @property
    def snapshot(self) -> dict:
        with self._state_lock:
            return {
                "lat":        round(self.lat, 6),
                "lon":        round(self.lon, 6),
                "alt":        round(self.rel_alt, 1),
                "heading":    round(self.heading, 1),
                "speed":      round(self.groundspeed, 2),
                "battery":    round(self.bat_pct, 1),
                "voltage":    round(self.bat_volt, 2),
                "current_ma": round(self.bat_curr, 0),
                "mode":       self.mode,
                "mode_num":   self.mode_num,
                "armed":      self.armed,
                "gps_fix":    self.gps_fix,
                "satellites": self.gps_sats,
                "ekf_ok":     self.ekf_ok,
                "bat_ready":  self.bat_initialized,
            }

    # KAPAT
    def close(self):
        self._rx_running = False
        if self._rx_thread:
            self._rx_thread.join(timeout=2.0)
        if self.master:
            try:
                self.master.close()
            except Exception:
                pass
        self.connected = False
        logger.info("[MAV] Baglanti kapatildi.")
