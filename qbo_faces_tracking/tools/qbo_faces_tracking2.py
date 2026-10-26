#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Détection de visages UltraFace RFB-320 ONNX sur vidéo locale.
Optimisé Jetson Orin NX (ONNXRuntime CUDA + OpenCV), 
affichage unique, FPS, code largement commenté.
"""

import cv2
import numpy as np
import onnxruntime as ort
import time
from math import ceil

# ======== PARAMÈTRES ========
VIDEO_PATH = "/home/nvidia/Downloads/VID_20250719_114809795_2.mp4"
MODEL_PATH = "/home/nvidia/qbo_ws/src/qbo_faces_tracking/tools/version-RFB-320.onnx"
IMG_W, IMG_H = 320, 240   # Taille attendue par le modèle UltraFace RFB-320
CONF_THRESH = 0.8
NMS_THRESH = 0.2
VARIANCES = [0.1, 0.2]
WINDOW_NAME = "UltraFace RFB-320 - Détection de visages"  # Fenêtre unique

# ======== FONCTIONS UTILITAIRES ========

def generate_anchors_ultraface_4420(img_w=320, img_h=240):
    """
    Génère exactement 4420 ancres pour UltraFace RFB-320 (input 320x240).
    """
    min_sizes = [
        [10, 16, 24],           # 30 x 40 x 3 = 3600
        [32, 48],               # 15 x 20 x 2 = 600
        [64, 96],               # 8 x 10 x 2 = 160
        [128, 192, 256]         # 4 x 5 x 3 = 60
    ]
    steps = [8, 16, 32, 64]
    feature_maps = [
        [ceil(img_h / 8), ceil(img_w / 8)],     # 30 x 40
        [ceil(img_h / 16), ceil(img_w / 16)],   # 15 x 20
        [ceil(img_h / 32), ceil(img_w / 32)],   # 8 x 10
        [ceil(img_h / 64), ceil(img_w / 64)]    # 4 x 5
    ]

    anchors = []
    for k, f in enumerate(feature_maps):
        for i in range(f[0]):
            for j in range(f[1]):
                for min_size in min_sizes[k]:
                    s_kx = min_size / img_w
                    s_ky = min_size / img_h
                    cx = (j + 0.5) * steps[k] / img_w
                    cy = (i + 0.5) * steps[k] / img_h
                    anchors.append([cx, cy, s_kx, s_ky])

    anchors = np.array(anchors, dtype=np.float32)
    print(f"[INFO] {anchors.shape[0]} ancres générées")
    return anchors

def preprocess(img):
    """Prétraitement UltraFace : resize, normalisation, format CHW float32"""
    img = cv2.resize(img, (IMG_W, IMG_H))
    img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
    img = (img.astype(np.float32) - 127.0) / 128.0
    img = np.transpose(img, (2, 0, 1))  # CHW
    return np.expand_dims(img, axis=0).astype(np.float32)

def decode_boxes(raw_boxes, anchors):
    """
    Décodage des boîtes normalisées UltraFace -> pixels sur image 320x240
    """
    boxes = np.concatenate([
        anchors[:, :2] + raw_boxes[:, :2] * VARIANCES[0] * anchors[:, 2:],
        anchors[:, 2:] * np.exp(raw_boxes[:, 2:] * VARIANCES[1])
    ], axis=1)
    boxes[:, :2] -= boxes[:, 2:] / 2
    boxes[:, 2:] += boxes[:, :2]
    boxes[:, [0, 2]] *= IMG_W
    boxes[:, [1, 3]] *= IMG_H
    return boxes

def iou(box, boxes):
    """Intersection over Union pour NMS"""
    inter = np.maximum(0, np.minimum(boxes[:, 2], box[2]) - np.maximum(boxes[:, 0], box[0])) * \
            np.maximum(0, np.minimum(boxes[:, 3], box[3]) - np.maximum(boxes[:, 1], box[1]))
    area1 = (box[2] - box[0]) * (box[3] - box[1])
    area2 = (boxes[:, 2] - boxes[:, 0]) * (boxes[:, 3] - boxes[:, 1])
    return inter / (area1 + area2 - inter + 1e-6)

def nms(boxes, scores, threshold=0.3):
    """Suppression Non Maximale (NMS)"""
    idxs = np.argsort(-scores)
    keep = []
    while len(idxs) > 0:
        i = idxs[0]
        keep.append(i)
        if len(idxs) == 1:
            break
        iou_vals = iou(boxes[i], boxes[idxs[1:]])
        idxs = idxs[1:][iou_vals < threshold]
    return keep

# ======== INITIALISATION MODELE ET VIDEO ========

print("[INFO] Chargement du modèle ONNX (UltraFace)...")
session = ort.InferenceSession(MODEL_PATH, providers=["CUDAExecutionProvider"])
input_name = session.get_inputs()[0].name
anchors = generate_anchors_ultraface_4420(IMG_W, IMG_H)

cap = cv2.VideoCapture(VIDEO_PATH)
if not cap.isOpened():
    print(f"[ERREUR] Impossible d'ouvrir la vidéo : {VIDEO_PATH}")
    exit(1)

# Pré-crée la fenêtre (évite clignotement OpenCV)
cv2.namedWindow(WINDOW_NAME, cv2.WINDOW_NORMAL)
cv2.resizeWindow(WINDOW_NAME, 800, 600)  # Taille affichage

# ======== BOUCLE PRINCIPALE ========

frame_id = 0
while True:
    ret, frame = cap.read()
    if not ret:
        break
    frame_id += 1
    start_time = time.time()

    # Pré-traitement pour UltraFace
    img_input = preprocess(frame)

    # Inference modèle UltraFace ONNX (output: conf, loc)
    conf, loc = session.run(None, {input_name: img_input})
    loc = loc[0]  # (4420, 4)
    conf = conf[0]  # (4420, 2)

    # Extraction des scores "face"
    face_scores = conf[:, 1] if conf.shape[1] >= 2 else conf[:, 0]
    keep = np.where(face_scores > CONF_THRESH)[0]

    # Calcul des ratios de redimensionnement (pour afficher les bboxes correctement)
    frame_h, frame_w = frame.shape[:2]
    scale_x = frame_w / IMG_W
    scale_y = frame_h / IMG_H

    if len(keep) > 0:
        boxes = decode_boxes(loc[keep], anchors[keep])
        scores = face_scores[keep]
        keep_nms = nms(boxes, scores, NMS_THRESH)
        for i in keep_nms:
            # Bounding box prédite (coordonnées 320x240)
            x1, y1, x2, y2 = boxes[i].astype(np.int32)
            # Upscale coordonnées à la taille d'affichage réelle (ex : 640x480)
            x1 = int(x1 * scale_x)
            x2 = int(x2 * scale_x)
            y1 = int(y1 * scale_y)
            y2 = int(y2 * scale_y)
            # Affichage rectangle vert + score
            cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
            cv2.putText(frame, f"{scores[i]:.2f}", (x1, max(y1 - 5, 0)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)

    # FPS calcul et affichage
    fps = 1.0 / (time.time() - start_time)
    cv2.putText(frame, f"FPS: {fps:.1f}", (10, 25),
                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 0, 0), 2)

    # Affichage unique dans une seule fenêtre
    cv2.imshow(WINDOW_NAME, frame)
    key = cv2.waitKey(1) & 0xFF
    if key == 27:   # ESC pour quitter
        break

cap.release()
cv2.destroyAllWindows()
print(f"[INFO] Terminé. {frame_id} frames traitées.")
