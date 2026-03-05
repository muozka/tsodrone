# 🚁 Drone Plaka Takip Sistemi

**Raspberry Pi 4 + Pixhawk 32Bit — Tam Proje**

---

## Proje Özeti

Polis araçlarının yükünü azaltmak için tasarlanmış otonom drone sistemi.
Drone kamera görüntüsü üzerinden plaka tespit eder, hedef aracı takip eder
ve tüm veriyi WiFi üzerinden WebSocket ile komuta merkezine iletir.

```
Kamera → YOLOv8 (Plaka Tespit) → PID Kontrolcü → MAVLink → Pixhawk → Motorlar
                                        ↓
                              WebSocket Server (ws://)
                                        ↓
                              Tarayıcı GCS Arayüzü
```

---

## Klasör Yapısı

```
drone_plaka_takip/
├── src/
│   ├── main.py              ← Ana program giriş noktası
│   ├── drone_controller.py  ← MAVLink / Pixhawk kontrolü
│   ├── alpr_detector.py     ← Plaka tespit (YOLO + OCR)
│   ├── vehicle_tracker.py   ← PID takip kontrolcüsü
│   ├── ws_server.py         ← WebSocket GCS sunucusu
│   └── telemetry.py         ← Telemetri monitörü
├── static/
│   └── index.html           ← Tarayıcı GCS arayüzü
├── config/
│   └── settings.py          ← Tüm ayarlar tek yerden
├── models/                  ← YOLO model dosyaları (.pt)
├── logs/                    ← Uçuş kayıtları
├── scripts/
│   ├── setup.sh             ← Tek komutla kurulum
│   ├── start.sh             ← Sistemi başlat
│   └── test_connection.py   ← Pixhawk bağlantı testi
├── tests/
│   └── test_alpr.py         ← ALPR birim testleri
├── requirements.txt         ← Python bağımlılıkları
└── README.md                ← Bu dosya
```

---

## Hızlı Başlangıç

```bash
# 1. Kurulum (tek seferlik)
chmod +x scripts/setup.sh
./scripts/setup.sh

# 2. Pixhawk bağlantısını test et
python3 scripts/test_connection.py

# 3. Sistemi başlat
./scripts/start.sh

# 4. Tarayıcıda arayüzü aç
# http://RPI_IP:8765
```

---

## Donanım Gereksinimleri

| Bileşen | Model |
|---------|-------|
| Bilgisayar | Raspberry Pi 4 Model B (8GB) |
| Uçuş Kontrolcüsü | Pixhawk 32Bit (STM32F7) |
| Kamera | RPi Camera Module 3 |
| GPS | M8N GPS + Compass |
| İletişim | UART: Pin8(TX)→Telem2-Pin3, Pin10(RX)←Telem2-Pin2, Pin6(GND)—Telem2-Pin6 |

---

## Lisans

Yalnızca eğitim ve araştırma amaçlıdır.
Operasyonel kullanım için SHY-İHA yönetmeliği kapsamında gerekli izinler alınmalıdır.
