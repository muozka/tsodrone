# Pixhawk 2.4.8 Optimizasyon Raporu
## drone_plaka_takip — ArduCopter Uyumluluk Analizi

---

## ÖZET

Orijinal proje **PX4 firmware** varsayımlarıyla yazılmıştı. Pixhawk 2.4.8 donanımı çoğunlukla **ArduCopter (ArduPilot)** firmware ile kullanılır ve bu iki firmware arasında kritik MAVLink protokol farkları vardır. Bu rapor tespit edilen tüm hataları ve uygulanan düzeltmeleri belgeler.

---

## KRİTİK HATALAR VE DÜZELTMELERİ

### 1. MOD NUMARALARI — En Önemli Hata

| | PX4 (ESKİ — YANLIŞ) | ArduCopter (YENİ — DOĞRU) |
|---|---|---|
| API | `set_mode(main_mode, sub_mode)` | `set_mode(mode_num)` |
| GUIDED/OFFBOARD | `PX4_MODE_OFFBOARD = (6, 0)` | `COPTER_MODE_GUIDED = 4` |
| RTL | `PX4_MODE_RTL = (4, 5)` | `COPTER_MODE_RTL = 6` |
| LAND | `PX4_MODE_LAND = (4, 6)` | `COPTER_MODE_LAND = 9` |
| LOITER | `PX4_MODE_LOITER = (4, 3)` | `COPTER_MODE_LOITER = 5` |
| TAKEOFF | `PX4_MODE_TAKEOFF = (4, 2)` | **Mod yok** → `MAV_CMD_NAV_TAKEOFF` |

**Sonuç:** Orijinal kodun RTL, LAND, mod geçişleri Pixhawk 2.4.8'de hiçbir şey yapmıyordu.

---

### 2. KALKIŞ PROSEDÜRÜ — Kritik Hata

**Orijinal (Yanlış — PX4):**
```python
self.set_mode(*PX4_MODE_POSCTL)   # POSCTL mod
self.arm()
self.set_mode(*PX4_MODE_TAKEOFF)  # PX4 TAKEOFF sub-mode
```

**Düzeltildi (ArduCopter):**
```python
self.set_mode_guided()             # GUIDED mod (şart)
self.arm()
self._send_command_long(           # MAV_CMD_NAV_TAKEOFF
    MAV_CMD_NAV_TAKEOFF,
    p7=altitude                    # hedef irtifa
)
```

**Sebep:** ArduCopter'da PX4'ün "TAKEOFF sub-mode" kavramı yoktur. Kalkış için GUIDED modda `MAV_CMD_NAV_TAKEOFF` komutu gönderilmesi gerekir.

---

### 3. ARM PROSEDÜRÜ — Davranış Farkı

**Orijinal (Kısmen Doğru):**
```python
self.master.set_mode('STABILIZE')
time.sleep(2)
self.master.arducopter_arm()      # Doğru fonksiyon ama eksik
```

**Düzeltildi:**
```python
self.set_mode_stabilize()         # MAV_CMD_DO_SET_MODE ile (tutarlı)
time.sleep(1.5)
MAV_CMD_COMPONENT_ARM_DISARM      # p1=1 (ARM), p2=0 (normal)
# Force ARM için: p2=2989 (ArduPilot magic number)
```

**Eklenendi:** Force ARM desteği — arming check bypass için `p2=2989`.

---

### 4. MANUEL HIZ KOMUTU — Yaw Rate Entegrasyonu

**Orijinal (Yanlış):**
```python
# Hız ve yaw ayrı komutlarla gönderiliyordu
send_ned_velocity(vx, vy, vz)      # hız
set_yaw(yaw_rate, relative=True)   # ayrı yaw komutu
```

**Düzeltildi:**
```python
# Yaw rate varsa tek komutla birleşik gönder
send_ned_velocity_with_yaw_rate(vx, vy, vz, yaw_rate_dps)
# SET_POSITION_TARGET_LOCAL_NED, type_mask=TYPEMASK_VEL_YAW_RATE
```

**Sebep:** `MAV_CMD_CONDITION_YAW` hız komutuyla aynı anda verilince ArduCopter'da çakışma olabilir. `SET_POSITION_TARGET_LOCAL_NED` içindeki `yaw_rate` alanı daha güvenilirdir.

---

### 5. MANUEL KONTROL MOD GEÇİŞİ — ws_server.py

**Orijinal (Eksik):**
```python
elif action == "manual_move":
    send_ned_velocity(vx, vy, vz)  # Mod kontrolü yok!
```

**Düzeltildi:**
```python
elif action == "manual_move":
    # GUIDED modda değilse önce geç
    if _real_drone._conn.mode_num != COPTER_MODE_GUIDED:
        _real_drone._conn.set_mode_guided()
    # Yaw rate varsa birleşik komut
    if yaw_rate != 0:
        send_ned_velocity_with_yaw_rate(vx, vy, vz, yaw_rate)
    else:
        send_ned_velocity(vx, vy, vz)
```

---

### 6. TELEMETRİ STREAM'LERİ — Bant Genişliği Optimizasyonu

**Orijinal (Verimsiz):**
```python
request_data_stream_send(..., MAV_DATA_STREAM_ALL, 50)  # 50 Hz her şey
```

**Düzeltildi (57600 baud uyumlu):**
```python
MAV_DATA_STREAM_RAW_SENSORS     → 10 Hz  (GPS, IMU)
MAV_DATA_STREAM_EXTENDED_STATUS → 10 Hz  (SYS_STATUS, pil)
MAV_DATA_STREAM_RC_CHANNELS     →  5 Hz  (RC giriş)
MAV_DATA_STREAM_POSITION        → 10 Hz  (GPS konum)
MAV_DATA_STREAM_EXTRA1          → 20 Hz  (ATTITUDE — en sık)
MAV_DATA_STREAM_EXTRA2          → 10 Hz  (VFR_HUD)
MAV_DATA_STREAM_EXTRA3          →  5 Hz  (AHRS)
```

**Sebep:** 57600 baud ~= 57600 bit/s ≈ 7200 byte/s. MAV_DATA_STREAM_ALL @ 50Hz bant genişliğini aşar, veri kayıplarına yol açar.

---

### 7. ACK KUYRUĞU — Komut Eşleştirme Düzeltmesi

**Orijinal (Hatalı):**
```python
ack = self._ack_queue.get(timeout=5.0)
# Hangi komutun ACK'i olduğu kontrol edilmiyordu!
```

**Düzeltildi:**
```python
while time.time() < deadline:
    ack = self._ack_queue.get(timeout=1.0)
    if ack.command == command:  # Komut eşleştirmesi
        return ack.result == MAV_RESULT_ACCEPTED
    # Farklı komutun ACK'i → geri koy
    self._ack_queue.put_nowait(ack)
```

---

### 8. EKF SAĞLIK MONİTÖRÜ — Yeni Özellik

`SYS_STATUS` mesajından kritik sensör sağlığı izleniyor:
```python
CRITICAL = (SENSOR_3D_GYRO | SENSOR_3D_ACCEL | SENSOR_GPS)
self.ekf_ok = (health & enabled_critical) == enabled_critical
```
`snapshot` içinde `ekf_ok: bool` olarak GCS'e iletilir.

---

### 9. VFR_HUD heading Tipi Düzeltmesi

**Orijinal:**
```python
self.heading = msg.heading  # int idi, float bekleniyor
```
**Düzeltildi:**
```python
self.heading = float(msg.heading)
```

---

### 10. STATUSTEXT Mesajları — Yeni Özellik

ArduCopter'ın sistem mesajları (EKF hatası, arming red, GPS uyarısı vb.) artık Python logger'a ve GCS loguna iletiliyor:
```
[FC] EKF2 IMU0 is using GPS
[FC] GPS: RTK Fixed
[FC] Arming: Compass not calibrated
```

---

## DOSYA DEĞİŞİKLİK ÖZETİ

| Dosya | Değişiklik |
|---|---|
| `src/mavlink_connection.py` | **Tamamen yeniden yazıldı** — ArduCopter mod numaraları, NAV_TAKEOFF, ACK eşleştirme, EKF sağlık, yaw_rate entegrasyonu, stream optimizasyonu |
| `src/drone_controller.py` | **Yeniden yazıldı** — enable_guided(), hover() güncellendi, eski PX4 alias'ları geriye dönük uyumluluk için korundu |
| `config/settings.py` | Stream hızları eklendi, telemetri Hz düşürüldü (57600 baud uyumu) |
| `src/ws_server.py` | manual_move GUIDED mod kontrolü, set_mode routing ArduCopter'a yönlendirildi |

---

## PIXHAWK 2.4.8 BAĞLANTI NOTLARI

### UART Port Seçimi
```
TELEM1 (/dev/ttyAMA0) → 57600 baud   ← ÖNERİLEN
TELEM2 (/dev/ttyAMA0) → 57600 baud
USB    (/dev/ttyACM0) → 115200 baud
```

### ArduCopter Parametre Ayarları (Mission Planner)
```
SERIAL1_BAUD  = 57    (57600)
SERIAL1_PROTOCOL = 2  (MAVLink 2)
BRD_SAFETYENABLE = 0  (Safety switch kapalı — companion computer)
ARMING_CHECK  = 0     (Test ortamında)  |  1 (Gerçek uçuşta)
WPNAV_SPEED   = 800   (8 m/s max yatay — settings.py MAX_SPEED_MPS ile uyumlu)
```

### Raspberry Pi UART Aktifleştirme
```bash
# /boot/config.txt
enable_uart=1
dtoverlay=disable-bt

# Disable serial console
sudo systemctl disable serial-getty@ttyAMA0.service
```

---

## UÇUŞ MOD AKIŞI

```
Bağlan → GPS lock bekle (min 6 sat) → STABILIZE ARM
       → GUIDED mod → TAKEOFF → LOITER (hover)
       → Manuel: GUIDED + send_ned_velocity (her 100ms)
       → Hover: send_ned_velocity(0,0,0) veya LOITER
       → Dönüş: RTL modu → otomatik iniş → DISARM
```

---

*Rapor tarihi: 2026-03-04*
