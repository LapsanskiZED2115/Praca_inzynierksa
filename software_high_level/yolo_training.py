from ultralytics import YOLO
import os

# =========================
# ŚCIEŻKI
# =========================
DATA_YAML = r"C:\Users\kamil\Desktop\prac inż\podejscie_2_ai_modele\data.yaml"

# =========================
# SPRAWDZENIE
# =========================
if not os.path.exists(DATA_YAML):
    raise FileNotFoundError(f"Nie znaleziono data.yaml: {DATA_YAML}")

# =========================
# MODEL STARTOWY
# =========================

model = YOLO("yolov8n.pt")

# =========================
# TRENING
# =========================
model.train(
    data=DATA_YAML,
    epochs=50,          
    imgsz=640,
    batch=16,
    name="tree_train5",  
    project=r"C:\Users\kamil\Desktop\prac inż\podejscie_2_ai_modele\runs\detect",
    device=0           
)