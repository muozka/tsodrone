#!/bin/bash
# ============================================================
#  DRONE KONTROL SİSTEMİ - Başlangıç Scripti
#  Kullanim: ./scripts/start.sh [--demo] [--sitl]
# ============================================================
set -e
GRN='\033[0;32m'
YLW='\033[1;33m'
CYN='\033[0;36m'
NC='\033[0m'

# Proje kokune git
cd "$(dirname "$0")/.."

# Sanal ortam varsa aktive et
if [ -f ".venv/bin/activate" ]; then
    source .venv/bin/activate
fi

echo ""
echo -e "${CYN}======================================================${NC}"
echo -e "${CYN}   DRONE KONTROL SİSTEMİ BAŞLATILIYOR${NC}"
echo -e "${CYN}   Arayüz Portu  : http://$(hostname -I | awk '{print $1}'):8765${NC}"
echo -e "${CYN}======================================================${NC}"
echo ""

# Ana programi calistir
exec python3 src/main.py "$@"

