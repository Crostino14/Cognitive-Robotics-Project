import os
import cv2
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
import time
from sklearn.metrics import roc_curve, auc

# --- CONFIGURAZIONE ---
BASE_DIR = os.path.dirname(os.path.abspath(__file__))

FACE_PROTO = os.path.join(BASE_DIR, "../scripts/Detector/FaceDetectorModels/opencv_face_detector.pbtxt")
FACE_MODEL = os.path.join(BASE_DIR, "../scripts/Detector/FaceDetectorModels/opencv_face_detector_uint8.pb")

FACE_SCORE_THRESHOLD = 0.5

# Percorsi dataset FDDB
FDDB_IMAGES_DIR = os.path.join(BASE_DIR, "TestSet/FDDB_Kaggle/originalPics")
FDDB_ANNOTATIONS_DIR = os.path.join(BASE_DIR, "TestSet/FDDB_Kaggle/FDDB-folds")


class FaceDetector:
    def __init__(self):
        self._faceNet = cv2.dnn.readNet(FACE_PROTO, FACE_MODEL)

        if cv2.cuda.getCudaEnabledDeviceCount() > 0:
            print("✅ OpenCV con CUDA rilevato. Attivando GPU.")
            self._faceNet.setPreferableBackend(cv2.dnn.DNN_BACKEND_CUDA)
            self._faceNet.setPreferableTarget(cv2.dnn.DNN_TARGET_CUDA)
        else:
            print("⚠️ CUDA non disponibile, si utilizzerà la CPU.")
            self._faceNet.setPreferableBackend(cv2.dnn.DNN_BACKEND_DEFAULT)
            self._faceNet.setPreferableTarget(cv2.dnn.DNN_TARGET_CPU)

    def detect_faces(self, image):
        start_time = time.time()  
        blob = cv2.dnn.blobFromImage(image, 1.0, (300, 300), [104, 117, 123], swapRB=True, crop=False)
        self._faceNet.setInput(blob)
        detections = self._faceNet.forward()
        inference_time = time.time() - start_time  

        bboxes = []
        confidences = []
        for i in range(detections.shape[2]):
            score = detections[0, 0, i, 2]
            if score > FACE_SCORE_THRESHOLD:
                box = detections[0, 0, i, 3:7] * np.array(
                    [image.shape[1], image.shape[0], image.shape[1], image.shape[0]]
                )
                box = box.astype(int)
                bboxes.append(box)
                confidences.append(score)  # ✅ Salviamo il punteggio di confidenza

        return bboxes, confidences, inference_time


# Conversione da Ellisse a Bounding Box
def ellipse_to_bbox(major_axis, minor_axis, center_x, center_y, angle):
    cos_angle = np.cos(angle)
    sin_angle = np.sin(angle)

    x1 = int(center_x - abs(major_axis * cos_angle) - abs(minor_axis * sin_angle))
    y1 = int(center_y - abs(major_axis * sin_angle) - abs(minor_axis * cos_angle))
    x2 = int(center_x + abs(major_axis * cos_angle) + abs(minor_axis * sin_angle))
    y2 = int(center_y + abs(major_axis * sin_angle) + abs(minor_axis * cos_angle))

    return [max(0, x1), max(0, y1), x2, y2]


# Lettura delle annotazioni da FDDB
def read_fddb_annotations(annotation_file):
    annotations = {}
    with open(annotation_file, 'r') as f:
        lines = f.read().strip().split('\n')

    idx = 0
    while idx < len(lines):
        image_rel_path = lines[idx].strip()
        idx += 1

        if idx >= len(lines) or not lines[idx].strip().isdigit():
            continue

        n_faces = int(lines[idx].strip())
        idx += 1  

        ground_truth_boxes = []
        for _ in range(n_faces):
            if idx >= len(lines):  
                break

            values = list(map(float, lines[idx].split()))
            if len(values) < 5:
                idx += 1
                continue 

            major_axis, minor_axis, angle, center_x, center_y = values[:5]
            bbox = ellipse_to_bbox(major_axis, minor_axis, center_x, center_y, angle)
            ground_truth_boxes.append(bbox)
            idx += 1  

        annotations[image_rel_path] = ground_truth_boxes

    return annotations


# Funzione per calcolare le metriche globali
def compute_global_metrics(tp, fp, fn, tn):
    precision = tp / (tp + fp) if (tp + fp) > 0 else 0
    recall = tp / (tp + fn) if (tp + fn) > 0 else 0
    accuracy = (tp + tn) / (tp + fp + tn + fn) if (tp + fp + tn + fn) > 0 else 0
    f1_score = 2 * (precision * recall) / (precision + recall) if (precision + recall) > 0 else 0
    return precision, recall, f1_score, accuracy


# Funzione per visualizzare la curva ROC
def plot_roc_curve(y_true, y_scores):
    if len(set(y_true)) < 2:  
        print("⚠️ Non abbastanza dati per generare la curva ROC")
        return

    fpr, tpr, _ = roc_curve(y_true, y_scores)
    roc_auc = auc(fpr, tpr)

    plt.figure(figsize=(6, 6))
    plt.plot(fpr, tpr, color='blue', lw=2, label=f'ROC curve (AUC = {roc_auc:.2f})')
    plt.plot([0, 1], [0, 1], color='gray', lw=2, linestyle='--')
    plt.xlabel('False Positive Rate')
    plt.ylabel('True Positive Rate')
    plt.title('Receiver Operating Characteristic (ROC) Curve')
    plt.legend(loc="lower right")
    plt.show()


##########################
# MAIN LOOP
##########################

face_detector = FaceDetector()
results = []

total_tp, total_fp, total_fn, total_tn = 0, 0, 0, 0
total_gt_faces, total_predicted_faces, total_inference_time = 0, 0, 0
y_true, y_scores = [], []

annotation_files = [os.path.join(FDDB_ANNOTATIONS_DIR, f) for f in os.listdir(FDDB_ANNOTATIONS_DIR) if "ellipseList.txt" in f]

for annotation_file in annotation_files:
    annotations = read_fddb_annotations(annotation_file)

    for image_rel_path, ground_truth_boxes in annotations.items():
        image_path = os.path.join(FDDB_IMAGES_DIR, image_rel_path + ".jpg")

        if not os.path.exists(image_path):
            continue

        image = cv2.imread(image_path)
        if image is None:
            continue

        predictions, confidences, inference_time = face_detector.detect_faces(image)

        total_gt_faces += len(ground_truth_boxes)
        total_predicted_faces += len(predictions)
        total_inference_time += inference_time

        tp = min(len(ground_truth_boxes), len(predictions))  
        fp = max(0, len(predictions) - tp)  
        fn = max(0, len(ground_truth_boxes) - tp)  
        tn = 0  

        total_tp += tp
        total_fp += fp
        total_fn += fn
        total_tn += tn

        y_true.extend([1] * tp + [0] * fp)
        y_scores.extend(confidences)

# 📊 Calcolo delle metriche globali
precision, recall, f1_score, accuracy = compute_global_metrics(total_tp, total_fp, total_fn, total_tn)

# 📊 Stampa delle metriche
print(f"✅ Accuracy: {accuracy:.3f}")
print(f"✅ Precision: {precision:.3f}")
print(f"✅ Recall: {recall:.3f}")
print(f"✅ F1-Score: {f1_score:.3f}")
print(f"✅ Tempo medio di inferenza: {total_inference_time / total_gt_faces:.4f} secondi per immagine")

# 📊 Traccia curva ROC
plot_roc_curve(y_true, y_scores)