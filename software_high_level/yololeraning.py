import random
import os
import numpy as np
import matplotlib.pyplot as plt

from ultralytics import YOLO
from sklearn.metrics import confusion_matrix, ConfusionMatrixDisplay

# =========================
# ŚCIEŻKI – POPRAWNE
# =========================
MODEL_PATH = r"C:\Users\kamil\Desktop\prac inż\podejscie_2_ai_modele\runs\detect\tree_train4\weights\best.pt"
TEST_IMAGES_PATH = r"C:\Users\kamil\Desktop\prac inż\podejscie_2_ai_modele\test\images"
TEST_LABELS_PATH = r"C:\Users\kamil\Desktop\prac inż\podejscie_2_ai_modele\test\labels"

NUM_SAMPLES = 100
CONF_THRESHOLD = 0.25

# =========================
# SPRAWDZENIA
# =========================
if not os.path.exists(MODEL_PATH):
    raise FileNotFoundError(f"Nie znaleziono modelu: {MODEL_PATH}")

if not os.path.exists(TEST_IMAGES_PATH):
    raise FileNotFoundError("Brak folderu test/images")

# =========================
# WCZYTANIE MODELU
# =========================
model = YOLO(MODEL_PATH)

# =========================
# LISTA ZDJĘĆ
# =========================
all_images = [
    f for f in os.listdir(TEST_IMAGES_PATH)
    if f.lower().endswith((".jpg", ".png", ".jpeg"))
]

print("Liczba obrazów testowych:", len(all_images))

if len(all_images) == 0:
    raise RuntimeError("Folder test/images jest pusty")

# =========================
# LOSOWANIE
# =========================
num_samples = min(NUM_SAMPLES, len(all_images))
selected_images = random.sample(all_images, num_samples)

# =========================
# EWALUACJA
# =========================
y_true = []
y_pred = []

for img_name in selected_images:
    img_path = os.path.join(TEST_IMAGES_PATH, img_name)
    label_path = os.path.join(
        TEST_LABELS_PATH,
        os.path.splitext(img_name)[0] + ".txt"
    )

    # Ground truth
    if os.path.exists(label_path):
        with open(label_path, "r") as f:
            gt = [int(line.split()[0]) for line in f.readlines()]
        gt_class = gt[0] if len(gt) > 0 else -1
    else:
        gt_class = -1

    # Predykcja
    results = model(img_path, conf=CONF_THRESHOLD, verbose=False)

    if results[0].boxes is not None and len(results[0].boxes) > 0:
        pred_class = int(results[0].boxes.cls[0].item())
    else:
        pred_class = -1

    y_true.append(gt_class)
    y_pred.append(pred_class)

# =========================
# MACIERZ KONFUZJI
# =========================
labels = sorted(set(y_true + y_pred))

cm = confusion_matrix(y_true, y_pred, labels=labels)
disp = ConfusionMatrixDisplay(confusion_matrix=cm, display_labels=labels)

disp.plot(cmap="Blues")
plt.title("Macierz konfuzji – YOLOv8")
plt.show()
