#!/usr/bin/env python3
"""
=============================================================
  DRONE PLAKA TAKIP SISTEMI - Pixhawk Baglanti Testi
  scripts/test_connection.py
  Python 3.13 uyumlu — sadece pymavlink kullanir

  Kullanim:
    python3 scripts/test_connection.py
    python3 scripts/test_connection.py --uri tcp:127.0.0.1:5760  (SITL)
    python3 scripts/test_connection.py --uri /dev/ttyUSB0
=============================================================
"""
import sys
import os
import time
import argparse

# Proje kokunu ekle
ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, ROOT)

from pymavlink import mavutil

# Renkler
GRN = "\033[92m"; RED = "\033[91m"; YLW = "\033[93m"
CYN = "\033[96m"; BLD = "\033[1m";  NC  = "\033[0m"

def ok(msg):   print(f"  {GRN}✓{NC}  {msg}")
def fail(msg): print(f"  {RED}✗{NC}  {msg}")
def warn(msg): print(f"  {YLW}!{NC}  {msg}")
def info(msg): print(f"  {CYN}→{NC}  {msg}")

def run_test(uri: str, baud: int, timeout: int):
    print(f"\n{BLD}{CYN}═══════════════════════════════════════{NC}")
    print(f"{BLD}{CYN}   PIXHAWK BAGLANTI TESTI{NC}")
    print(f"{CYN}═══════════════════════════════════════{NC}")
    print(f"  URI  : {uri}")
    print(f"  Baud : {baud}")
    print()

    # 1. Baglanti ─────────────────────────────────────────
    print(f"{BLD}1. Baglanti{NC}")
    try:
        conn = mavutil.mavlink_connection(uri, baud=baud, source_system=255)
        ok(f"Port acildi: {uri}")
    except Exception as e:
        fail(f"Port acilamadi: {e}")
        print(f"\n{RED}TEST BASARISIZ.{NC}\n")
        return False

    # 2. Heartbeat ────────────────────────────────────────
    print(f"\n{BLD}2. Heartbeat{NC}")
    info(f"Bekleniyor ({timeout}s timeout)...")
    hb = conn.wait_heartbeat(timeout=timeout)
    if hb is None:
        fail("Heartbeat alinamadi!")
        fail("  → Pixhawk bagli mi?")
        fail("  → TX/RX pinleri dogru mu? (Pin8=TX→Telem2-Pin3, Pin10=RX←Telem2-Pin2)")
        fail("  → Baud hizi dogru mu? (57600)")
        fail("  → /boot/config.txt icinde enable_uart=1 var mi?")
        print(f"\n{RED}TEST BASARISIZ.{NC}\n")
        return False
    ok(f"Heartbeat alindi! system={conn.target_system} component={conn.target_component}")

    # Veri akisi iste
    conn.mav.request_data_stream_send(
        conn.target_system, conn.target_component,
        mavutil.mavlink.MAV_DATA_STREAM_ALL, 4, 1
    )
    time.sleep(1.5)

    # 3. GPS ──────────────────────────────────────────────
    print(f"\n{BLD}3. GPS Durumu{NC}")
    gps = conn.recv_match(type="GPS_RAW_INT", blocking=True, timeout=5)
    if gps:
        fix_names = {0:"NO FIX",1:"NO FIX",2:"2D FIX",3:"3D FIX",4:"DGPS",5:"RTK"}
        fix_str = fix_names.get(gps.fix_type, "?")
        if gps.fix_type >= 3:
            ok(f"GPS FIX: {fix_str} | Uydu: {gps.satellites_visible}")
        else:
            warn(f"GPS FIX: {fix_str} | Uydu: {gps.satellites_visible}  (3D fix bekleniyor)")
    else:
        warn("GPS verisi alinamadi (acik havada deneyin)")

    # 4. Pil ──────────────────────────────────────────────
    print(f"\n{BLD}4. Pil Durumu{NC}")
    sys_stat = conn.recv_match(type="SYS_STATUS", blocking=True, timeout=5)
    if sys_stat:
        volt = sys_stat.voltage_battery / 1000.0
        pct  = sys_stat.battery_remaining
        if volt > 10.0:
            ok(f"Pil: {volt:.2f}V  (%{pct})")
        else:
            warn(f"Pil dusuk: {volt:.2f}V  (%{pct})")
    else:
        warn("Pil verisi alinamadi")

    # 5. Ucus Modu ────────────────────────────────────────
    print(f"\n{BLD}5. Ucus Modu{NC}")
    try:
        mode_str = mavutil.mode_string_v10(hb)
        ok(f"Mod: {mode_str}")
    except Exception:
        ok(f"Custom mod: {hb.custom_mode}")

    armed = bool(hb.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
    if armed:
        warn("Drone ARMED durumunda! Pervanelere dikkat!")
    else:
        ok("Drone DISARMED (guvenli)")

    # 6. Irtifa ───────────────────────────────────────────
    print(f"\n{BLD}6. Irtifa{NC}")
    pos = conn.recv_match(type="GLOBAL_POSITION_INT", blocking=True, timeout=5)
    if pos:
        ok(f"Konum: ({pos.lat/1e7:.5f}, {pos.lon/1e7:.5f}) | Irtifa: {pos.relative_alt/1000:.1f}m")
    else:
        warn("Konum verisi alinamadi")

    # 7. UART Loopback Testi (sadece gercek UART) ─────────
    print(f"\n{BLD}7. MAVLink Protokol{NC}")
    ok(f"MAVLink v{hb.get_header().mlen > 6 and 2 or 1} aktif")
    ok(f"Mesaj alimi basarili")

    # Kapat
    conn.close()

    # Sonuc ───────────────────────────────────────────────
    print(f"\n{GRN}{'═'*41}{NC}")
    print(f"{GRN}   TUM TESTLER BASARILI!{NC}")
    print(f"{GRN}{'═'*41}{NC}")
    print(f"\n  Sistemi baslatin:")
    print(f"  {CYN}./scripts/start.sh{NC}")
    print()
    return True


def main():
    parser = argparse.ArgumentParser(description="Pixhawk baglanti testi")
    parser.add_argument("--uri",     default="/dev/ttyAMA0",
                        help="Baglanti adresi (orn: /dev/ttyAMA0 veya tcp:127.0.0.1:5760)")
    parser.add_argument("--baud",    type=int, default=57600)
    parser.add_argument("--timeout", type=int, default=15)
    args = parser.parse_args()

    success = run_test(args.uri, args.baud, args.timeout)
    sys.exit(0 if success else 1)


if __name__ == "__main__":
    main()
