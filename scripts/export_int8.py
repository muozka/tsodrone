"""
scripts/export_int8.py

YOLOv8 modelini INT8 ONNX formatına dönüştür.
Raspberry Pi 4 için optimize: 416x416 giriş, INT8 quantization.

Kullanım:
    python3 scripts/export_int8.py --model models/yolov8n_plate.pt
    python3 scripts/export_int8.py --model models/yolov8n_plate.pt --size 416

Çıktı:
    models/yolov8n_plate_int8.onnx
"""
import argparse
import os
import sys
import logging

logging.basicConfig(level=logging.INFO, format="%(message)s")
logger = logging.getLogger("export")

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, ROOT)


def export_int8_onnx(model_path: str, imgsz: int = 416, data: str = None):
    """
    YOLOv8 INT8 ONNX dışa aktarımı.

    INT8 quantization için calibration dataset gereklidir.
    data argümanı verilmezse FP16 ONNX dışa aktarılır
    (INT8'den biraz yavaş ama calibration dataseti gerekmez).
    """
    try:
        from ultralytics import YOLO
    except ImportError:
        logger.error("ultralytics kurulu değil: pip install ultralytics")
        sys.exit(1)

    if not os.path.exists(model_path):
        logger.error(f"Model bulunamadı: {model_path}")
        sys.exit(1)

    model = YOLO(model_path)

    logger.info(f"Model yüklendi: {model_path}")
    logger.info(f"Giriş boyutu: {imgsz}x{imgsz}")

    # Çıktı yolu
    base    = model_path.replace(".pt", "").replace(".onnx", "")
    out_path = base + "_int8.onnx"

    try:
        if data:
            # INT8 quantization (calibration dataset ile)
            logger.info("INT8 quantization başlıyor (calibration dataset kullanılıyor)...")
            model.export(
                format="onnx",
                imgsz=imgsz,
                int8=True,
                data=data,
                dynamic=False,
                simplify=True,
            )
            logger.info(f"INT8 ONNX dışa aktarıldı!")
        else:
            # FP16 ONNX (calibration dataseti olmadan)
            logger.info("FP16 ONNX dışa aktarılıyor (calibration dataset yok)...")
            model.export(
                format="onnx",
                imgsz=imgsz,
                half=True,          # FP16
                dynamic=False,
                simplify=True,
                opset=12,
            )
            logger.info("FP16 ONNX dışa aktarıldı")

        # Dosyayı isimlendir
        default_out = model_path.replace(".pt", ".onnx")
        if os.path.exists(default_out) and default_out != out_path:
            os.rename(default_out, out_path)
            logger.info(f"Yeniden adlandırıldı → {out_path}")

        # Boyut raporu
        if os.path.exists(out_path):
            size_mb = os.path.getsize(out_path) / (1024 * 1024)
            logger.info(f"Çıktı: {out_path} ({size_mb:.1f} MB)")
        else:
            # ultralytics farklı isim koymuş olabilir
            logger.warning(f"Çıktı dosyası {out_path} bulunamadı")
            # Yakın dosyaları tara
            model_dir = os.path.dirname(model_path) or "."
            for f in os.listdir(model_dir):
                if f.endswith(".onnx"):
                    logger.info(f"Bulunan ONNX: {os.path.join(model_dir, f)}")

    except Exception as e:
        logger.error(f"Dışa aktarım hatası: {e}")
        import traceback
        traceback.print_exc()
        sys.exit(1)


def benchmark(model_path: str, imgsz: int = 416, runs: int = 50):
    """Basit benchmark: ortalama inference süresi."""
    import time, numpy as np

    try:
        from ultralytics import YOLO
        model = YOLO(model_path, task="detect")
    except Exception as e:
        logger.error(f"Model yüklenemedi: {e}")
        return

    dummy = np.random.randint(0, 255, (imgsz, imgsz, 3), dtype=np.uint8)

    # Warmup
    for _ in range(5):
        model(dummy, verbose=False)

    # Benchmark
    times = []
    for _ in range(runs):
        t = time.perf_counter()
        model(dummy, verbose=False)
        times.append((time.perf_counter() - t) * 1000)

    avg = sum(times) / len(times)
    mn  = min(times)
    mx  = max(times)
    logger.info(
        f"Benchmark ({runs} run): "
        f"ort={avg:.1f}ms  min={mn:.1f}ms  max={mx:.1f}ms  "
        f"→ {1000/avg:.1f} FPS"
    )


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="YOLOv8 INT8/FP16 ONNX dışa aktarımı")
    parser.add_argument("--model",     default="models/yolov8n_plate.pt",
                        help="Kaynak model yolu (.pt)")
    parser.add_argument("--size",      type=int, default=416,
                        help="Giriş boyutu (varsayılan: 416)")
    parser.add_argument("--data",      default=None,
                        help="INT8 calibration dataset YAML (opsiyonel)")
    parser.add_argument("--benchmark", action="store_true",
                        help="Dışa aktarım sonrası benchmark çalıştır")
    args = parser.parse_args()

    export_int8_onnx(args.model, args.size, args.data)

    if args.benchmark:
        # INT8 ve orijinal modeli kıyasla
        out = args.model.replace(".pt", "_int8.onnx")
        if os.path.exists(out):
            logger.info(f"\n── INT8 ONNX Benchmark ──")
            benchmark(out, args.size)
        if os.path.exists(args.model):
            logger.info(f"\n── Orijinal PT Benchmark ──")
            benchmark(args.model, args.size)
