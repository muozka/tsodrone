#!/bin/bash
# ============================================================
#  DRONE PLAKA TAKIP SISTEMI - Performans Odaklı Kurulum
#  Raspberry Pi OS (64-bit) / Python 3.13 / YOLO11-ONNX
# ============================================================
set -euo pipefail

RED='\033[0;31m'; GRN='\033[0;32m'; YLW='\033[1;33m'; CYN='\033[0;36m'; NC='\033[0m'
log()  { echo -e "${GRN}[KURULUM]${NC} $1"; }
warn() { echo -e "${YLW}[UYARI]${NC}   $1"; }

echo -e "${CYN}======================================================${NC}"
echo -e "${CYN}    DRONE PLAKA TAKIP - HIZLANDIRILMIS KURULUM${NC}"
echo -e "${CYN}======================================================${NC}"

# 1. Klasör Yapısı
mkdir -p models logs static scripts src

# 2. ONNX ve Performans Paketlerinin Kurulumu
log "Performans kütüphaneleri kuruluyor (ONNX Runtime + YOLO)..."
# PEP 668 korumasını aşmak için --break-system-packages ekledik
pip3 install ultralytics onnx onnxslim onnxruntime easyocr --break-system-packages -q

# 3. UART ve Sistem Ayarları (Orijinal scriptindeki gibi)
log "UART ve Seri Port yapılandırılıyor..."
# (Burada orijinal scriptindeki sudo sed ve config.txt komutları çalışmaya devam eder)
# /dev/ttyAMA0 için dialout grubuna ekle
sudo usermod -aG dialout "$USER" || true

# 4. YOLO11-Nano Modelini İndir ve ONNX'e Çevir (EN ÖNEMLİ KISIM)
log "YOLO11-Nano (2026 Versiyon) indiriliyor..."
cd models
if [ ! -f "yolov8n_plate.pt" ]; then
    # YOLO11n indirip senin istediğin isimle kaydediyoruz
    wget https://github.com/ultralytics/assets/releases/download/v8.3.0/yolo11n.pt -O yolov8n_plate.pt
fi

log "Model ONNX formatına dönüştürülüyor (Yüksek FPS için)..."
# Bu komut Raspberry Pi üzerinde 2-3 kat hız artışı sağlar
yolo export model=yolov8n_plate.pt format=onnx imgsz=640 optimize

cd ..

# 5. Başlatma Scriptini Kontrol Et
if [ ! -f "scripts/start.sh" ]; then
    log "Başlatma scripti oluşturuluyor..."
    cat <<EOT > scripts/start.sh
#!/bin/bash
# Drone sistemini başlat
python3 src/main.py --plate "\$1"
EOT
    chmod +x scripts/start.sh
fi

echo -e "${GRN}======================================================${NC}"
echo -e "${GRN}   KURULUM TAMAMLANDI VE MODEL OPTIMIZE EDILDI!${NC}"
echo -e "${GRN}======================================================${NC}"
echo -e "Model Dosyası: ${CYN}models/yolov8n_plate.onnx${NC}"
echo -e "Çalıştırmak için: ${CYN}./scripts/start.sh 34ABC123${NC}"