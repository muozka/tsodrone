"""
src/processes/mavlink_proc.py

MAVLink process — Pixhawk 2.4.8 / ArduCopter

Sorumluluğu:
  1. DroneController üzerinden Pixhawk ile iletişim
  2. command_queue'dan komutları alıp uygula
  3. telemetry_queue'ya telemetri snapshotu yayınla
  4. WATCHDOG: main process'ten belirli süre heartbeat gelmezse → RTL

Watchdog mantığı:
  main process her 0.5s'de bir {"cmd": "heartbeat"} gönderir.
  Bu süre WATCHDOG_TIMEOUT_SEC aşılırsa companion crash kabul edilir
  ve drone otomatik RTL moduna alınır.
"""
import os
import sys
import time
import logging
import multiprocessing as mp

ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.insert(0, ROOT)


def mavlink_process_main(
    command_queue:  mp.Queue,
    telemetry_queue: mp.Queue,
    log_queue:      mp.Queue,
    watchdog_ts:    mp.Value,   # ctypes.c_double — son heartbeat zamanı
    uri:            str,
    baud:           int,
):
    """
    MAVLink process giriş noktası.
    Bu fonksiyon spawn ile ayrı process olarak çalıştırılır.
    """
    # Process'e özel loglama — log_queue üzerinden iletilir
    _setup_proc_logging(log_queue, "mavlink_proc")
    logger = logging.getLogger("mavlink_proc")
    logger.info(f"[MAVPROC] Başladı — {uri} @ {baud}")

    # DroneController başlat
    try:
        from src.drone_controller import DroneController
        from src.mavlink_connection import COPTER_MODE_GUIDED
        from config.settings import MAVLink, FailSafe
    except Exception as e:
        logger.error(f"[MAVPROC] Import hatası: {e}")
        return

    drone = DroneController()
    connected = drone.connect()
    if not connected:
        logger.error("[MAVPROC] Pixhawk bağlanamadı!")
        # Bağlantı yoksa telemetri queue'ya hata bildir
        try:
            telemetry_queue.put_nowait({"error": "no_connection"})
        except Exception:
            pass
        # Yine de komutları dinlemeye devam et (reconnect için)
    else:
        logger.info("[MAVPROC] Pixhawk bağlı")

    # Watchdog thread başlat
    import threading
    watchdog_active = threading.Event()
    watchdog_active.set()

    def _watchdog():
        while watchdog_active.is_set():
            time.sleep(1.0)
            if not connected:
                continue
            now = time.time()
            with watchdog_ts.get_lock():
                last_hb = watchdog_ts.value
            if last_hb > 0 and (now - last_hb) > MAVLink.WATCHDOG_TIMEOUT_SEC:
                logger.critical(
                    f"[MAVPROC] WATCHDOG: {now - last_hb:.1f}s heartbeat yok! "
                    f"→ RTL tetikleniyor"
                )
                try:
                    drone._conn.return_to_launch()
                    with watchdog_ts.get_lock():
                        watchdog_ts.value = now   # tekrar tetiklenmesin
                except Exception as ex:
                    logger.error(f"[MAVPROC] RTL hatası: {ex}")

    wdog_thread = threading.Thread(target=_watchdog, daemon=True, name="Watchdog")
    wdog_thread.start()

    # Telemetri yayın thread
    def _telem_publisher():
        while True:
            try:
                if drone.connected:
                    snap = drone.telemetry
                    # Non-blocking put — eski veriyi sil
                    try:
                        telemetry_queue.put_nowait(snap)
                    except Exception:
                        try:
                            telemetry_queue.get_nowait()
                            telemetry_queue.put_nowait(snap)
                        except Exception:
                            pass
                time.sleep(0.1)   # 10 Hz
            except Exception as e:
                logger.debug(f"[MAVPROC] Telemetri yayın hatası: {e}")

    telem_thread = threading.Thread(target=_telem_publisher, daemon=True, name="TelemPub")
    telem_thread.start()

    # ── Ana komut döngüsü ─────────────────────────────────────
    logger.info("[MAVPROC] Komut döngüsü aktif")
    while True:
        try:
            cmd = command_queue.get(timeout=1.0)
        except Exception:
            continue  # timeout — devam et

        if cmd is None:
            logger.info("[MAVPROC] Kapatma sinyali alındı")
            break

        action = cmd.get("cmd", "")

        try:
            if action == "heartbeat":
                with watchdog_ts.get_lock():
                    watchdog_ts.value = time.time()

            elif action == "velocity" and drone.connected:
                vx       = float(cmd.get("vx", 0))
                vy       = float(cmd.get("vy", 0))
                vz       = float(cmd.get("vz", 0))
                yaw_rate = float(cmd.get("yaw_rate", 0))
                if yaw_rate != 0:
                    drone._conn.send_ned_velocity_with_yaw_rate(vx, vy, vz, yaw_rate)
                else:
                    drone._conn.send_ned_velocity(vx, vy, vz)

            elif action == "arm" and drone.connected:
                drone._conn.arm()
                logger.info("[MAVPROC] ARM komutu uygulandı")

            elif action == "disarm" and drone.connected:
                drone.disarm()

            elif action == "takeoff" and drone.connected:
                alt = float(cmd.get("altitude", 15.0))
                drone.arm_and_takeoff(alt)

            elif action == "rtl" and drone.connected:
                drone.return_to_launch()

            elif action == "land" and drone.connected:
                drone.land()

            elif action == "set_mode_guided" and drone.connected:
                drone._conn.set_mode_guided()

            elif action == "set_mode_loiter" and drone.connected:
                drone._conn.set_mode_loiter()

            elif action == "set_mode_stabilize" and drone.connected:
                drone._conn.set_mode_stabilize()

            elif action == "set_mode_rtl" and drone.connected:
                drone._conn.set_mode_rtl()

            elif action == "set_mode_land" and drone.connected:
                drone._conn.set_mode_land()

            elif action == "set_mode" and drone.connected:
                mode_upper = cmd.get("mode", "").upper()
                if mode_upper in ("MANUEL", "GUIDED"):
                    drone._conn.set_mode_guided()
                elif mode_upper in ("OTOMATIK", "LOITER"):
                    drone._conn.set_mode_loiter()
                elif mode_upper == "RTL":
                    drone._conn.set_mode_rtl()
                elif mode_upper == "LAND":
                    drone._conn.set_mode_land()
                elif mode_upper == "STABILIZE":
                    drone._conn.set_mode_stabilize()

            elif action == "motor_test" and drone.connected:
                drone._conn.motor_test(
                    motor_id     = int(cmd.get("motor_id", 0)),
                    throttle_pct = float(cmd.get("throttle", 10)),
                    duration_sec = float(cmd.get("duration", 3)),
                )

            elif action == "rc_throttle" and drone.connected:
                drone._conn.rc_override_throttle(int(cmd.get("pwm", 1000)))

            elif action == "rc_release" and drone.connected:
                drone._conn.rc_override_release()

            elif action == "yaw" and drone.connected:
                drone._conn.set_yaw(
                    float(cmd.get("heading", 0)),
                    bool(cmd.get("relative", True)),
                    float(cmd.get("speed_dps", 20)),
                )

        except Exception as e:
            logger.error(f"[MAVPROC] Komut hatası [{action}]: {e}")

    # Temizlik
    watchdog_active.clear()
    if drone.connected:
        drone.disconnect()
    logger.info("[MAVPROC] Kapatıldı")


def _setup_proc_logging(log_queue: mp.Queue, proc_name: str):
    """Process içi loglama — QueueHandler ile ana process'e iletilir."""
    from logging.handlers import QueueHandler
    root = logging.getLogger()
    root.handlers.clear()
    root.addHandler(QueueHandler(log_queue))
    root.setLevel(logging.DEBUG)
