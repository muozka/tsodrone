#!/usr/bin/env python3
"""
Tek Motor Test - Guvenilir ARM + RC Override
"""
import sys, time, threading
from pymavlink import mavutil

PORT = "/dev/ttyAMA0"
BAUD = 57600
THROTTLE_PWM = 1000
RUNNING = True

def rc_sender(master):
    global THROTTLE_PWM, RUNNING
    while RUNNING:
        try:
            master.mav.rc_channels_override_send(
                master.target_system, master.target_component,
                1500, 1500, THROTTLE_PWM, 1500, 0, 0, 0, 0,
            )
        except: pass
        time.sleep(0.05)

def get_armed(master):
    for _ in range(20):
        msg = master.recv_match(type='HEARTBEAT', blocking=True, timeout=1)
        if msg and msg.get_srcComponent() == 1:
            return (msg.base_mode & 128) != 0
    return False

def arm_with_retry(master, max_attempts=5):
    for attempt in range(1, max_attempts + 1):
        while master.recv_match(blocking=False): pass
        print(f"  Deneme {attempt}/{max_attempts}...")
        master.set_mode('STABILIZE')
        time.sleep(2)
        while master.recv_match(blocking=False): pass
        master.arducopter_arm()
        t0 = time.time()
        while time.time() - t0 < 5:
            msg = master.recv_match(blocking=True, timeout=1)
            if msg:
                if msg.get_type() == 'HEARTBEAT' and msg.get_srcComponent() == 1:
                    if (msg.base_mode & 128) != 0:
                        print(f"  ARMED! (deneme {attempt})")
                        return True
                elif msg.get_type() == 'STATUSTEXT':
                    print(f"  Pixhawk: {msg.text}")
        print(f"  Bekleniyor...")
        time.sleep(3)
    return False

print("Baglaniliyor...")
master = mavutil.mavlink_connection(PORT, baud=BAUD, source_system=255, source_component=190)
master.wait_heartbeat()
print("Pixhawk OK")

print("\nBaglanti bekleniyor (5sn)...")
time.sleep(5)

print("\nARM ediliyor (5 deneme hakki)...")
if not arm_with_retry(master):
    print("ARM basarisiz! Pixhawk'i yeniden baslat ve tekrar dene.")
    sys.exit(1)

sender = threading.Thread(target=rc_sender, args=(master,), daemon=True)
sender.start()

print(f"""
{'='*40}
  MOTOR KONTROL
  Sayi (1000-1700) = PWM
  1000 = durdur | q = cikis
  Motor esigi: ~1350
{'='*40}
""")

while True:
    try:
        c = input(f"PWM [{THROTTLE_PWM}]: ").strip().lower()
    except (KeyboardInterrupt, EOFError):
        break
    if c == "q": break
    try: pwm = int(c)
    except: continue
    THROTTLE_PWM = max(1000, min(1700, pwm))
    print(f"  Throttle: {THROTTLE_PWM}")

print("\nDurdurma...")
THROTTLE_PWM = 1000
RUNNING = False
time.sleep(1)
master.mav.rc_channels_override_send(
    master.target_system, master.target_component, 0,0,0,0,0,0,0,0)
master.arducopter_disarm()
time.sleep(1)
print("Bitti.")
