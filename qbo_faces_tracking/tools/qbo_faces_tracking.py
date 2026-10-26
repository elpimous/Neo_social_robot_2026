#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Détection de visages UltraFace RFB-320 TensorRT (.engine) sur vidéo locale.
Compatible TensorRT 10+, optimisé Jetson Orin NX (tensorrt + pycuda).
Affichage unique, bbox upscaling, pipeline ROS2-friendly.
"""

import cv2
import numpy as np
import tensorrt as trt
import pycuda.driver as cuda
import pycuda.autoinit
import time
from math import ceil

# ======== PARAMÈTRES ========
VIDEO_PATH = "/home/nvidia/Downloads/VID_20250719_114809795_2_640x480.mp4"
ENGINE_PATH = "/home/nvidia/qbo_ws/src/qbo_faces_tracking/tools/version-RFB-320.engine"
IMG_W, IMG_H = 320, 240   # UltraFace RFB-320
CONF_THRESH = 0.85
NMS_THRESH = 0.3
VARIANCES = [0.1, 0.2]
WINDOW_NAME = "UltraFace RFB-320 - TRT"

# ======== FONCTIONS UTILES ========

def generate_anchors_ultraface_4420(img_w=320, img_h=240):
    min_sizes = [
        [10, 16, 24],
        [32, 48],
        [64, 96],
        [128, 192, 256]
    ]
    steps = [8, 16, 32, 64]
    feature_maps = [
        [ceil(img_h / 8), ceil(img_w / 8)],
        [ceil(img_h / 16), ceil(img_w / 16)],
        [ceil(img_h / 32), ceil(img_w / 32)],
        [ceil(img_h / 64), ceil(img_w / 64)]
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
    img = cv2.resize(img, (IMG_W, IMG_H))
    img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
    img = (img.astype(np.float32) - 127.0) / 128.0
    img = np.transpose(img, (2, 0, 1))
    return np.expand_dims(img, axis=0).astype(np.float32)

def decode_boxes(raw_boxes, anchors):
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
    inter = np.maximum(0, np.minimum(boxes[:, 2], box[2]) - np.maximum(boxes[:, 0], box[0])) * \
            np.maximum(0, np.minimum(boxes[:, 3], box[3]) - np.maximum(boxes[:, 1], box[1]))
    area1 = (box[2] - box[0]) * (box[3] - box[1])
    area2 = (boxes[:, 2] - boxes[:, 0]) * (boxes[:, 3] - boxes[:, 1])
    return inter / (area1 + area2 - inter + 1e-6)

def nms(boxes, scores, threshold=0.3):
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

# ======== INITIALISATION TENSORRT ========

print("[INFO] Chargement de l'engine TensorRT...")
TRT_LOGGER = trt.Logger(trt.Logger.WARNING)
with open(ENGINE_PATH, 'rb') as f:
    engine_data = f.read()
runtime = trt.Runtime(TRT_LOGGER)
engine = runtime.deserialize_cuda_engine(engine_data)
context = engine.create_execution_context()

# ======== IDENTIFICATION AUTOMATIQUE DES BINDINGS ========

# Liste les bindings et leurs shapes
binding_names = [engine.get_tensor_name(i) for i in range(engine.num_io_tensors)]
binding_shapes = [engine.get_tensor_shape(name) for name in binding_names]
is_input = [engine.get_tensor_mode(name) == trt.TensorIOMode.INPUT for name in binding_names]
print("[DEBUG] Liste des bindings (nom, shape, INPUT?):")
for i, name in enumerate(binding_names):
    print(f"  {i}: {name} | {binding_shapes[i]} | {'INPUT' if is_input[i] else 'OUTPUT'}")

# Trouve le nom de l'input et des 2 outputs automatiquement
input_name = None
conf_name = None
loc_name = None

for i, name in enumerate(binding_names):
    shp = binding_shapes[i]
    if is_input[i]:
        input_name = name
    else:
        # Shape (1,4420,2) -> conf; (1,4420,4) -> loc
        if shp[-1] == 2:
            conf_name = name
        elif shp[-1] == 4:
            loc_name = name

if not input_name or not conf_name or not loc_name:
    raise RuntimeError("Impossible d'inférer les noms des tensors dans l'engine !")

print(f"[INFO] Tensors utilisés : input={input_name}, conf={conf_name}, loc={loc_name}")

# ======== ALLOCATION MÉMOIRE (TYPE INT OBLIGATOIRE) ========
input_shape = engine.get_tensor_shape(input_name)
conf_shape = engine.get_tensor_shape(conf_name)
loc_shape = engine.get_tensor_shape(loc_name)

input_host = cuda.pagelocked_empty(int(np.prod(input_shape)), dtype=np.float32)
input_dev = cuda.mem_alloc(input_host.nbytes)
output_host_conf = cuda.pagelocked_empty(int(np.prod(conf_shape)), dtype=np.float32)
output_dev_conf = cuda.mem_alloc(output_host_conf.nbytes)
output_host_loc = cuda.pagelocked_empty(int(np.prod(loc_shape)), dtype=np.float32)
output_dev_loc = cuda.mem_alloc(output_host_loc.nbytes)
stream = cuda.Stream()

output_shapes = [conf_shape, loc_shape]  # Pour reshape

def infer_trt(img):
    img_input = preprocess(img).flatten().astype(np.float32)
    np.copyto(input_host, img_input.ravel())
    cuda.memcpy_htod_async(input_dev, input_host, stream)
    # Assignation automatique par nom (TensorRT 10+)
    context.set_tensor_address(input_name, int(input_dev))
    context.set_tensor_address(conf_name, int(output_dev_conf))
    context.set_tensor_address(loc_name, int(output_dev_loc))
    context.execute_async_v3(stream_handle=stream.handle)
    cuda.memcpy_dtoh_async(output_host_conf, output_dev_conf, stream)
    cuda.memcpy_dtoh_async(output_host_loc, output_dev_loc, stream)
    stream.synchronize()
    conf = output_host_conf.reshape(conf_shape)
    loc = output_host_loc.reshape(loc_shape)
    return conf, loc

# ======== ANCRES ULTRAFACE ========
anchors = generate_anchors_ultraface_4420(IMG_W, IMG_H)

# ======== LECTURE VIDEO ET AFFICHAGE ========
cap = cv2.VideoCapture(VIDEO_PATH)
if not cap.isOpened():
    print(f"[ERREUR] Impossible d'ouvrir la vidéo : {VIDEO_PATH}")
    exit(1)

cv2.namedWindow(WINDOW_NAME, cv2.WINDOW_NORMAL)
cv2.resizeWindow(WINDOW_NAME, 800, 600)

frame_id = 0
while True:
    ret, frame = cap.read()
    if not ret:
        break
    frame_id += 1
    start_time = time.time()

    # === Inférence TensorRT ===
    conf, loc = infer_trt(frame)
    conf = conf[0]
    loc = loc[0]

    face_scores = conf[:, 1] if conf.shape[1] >= 2 else conf[:, 0]
    keep = np.where(face_scores > CONF_THRESH)[0]

    frame_h, frame_w = frame.shape[:2]
    scale_x = frame_w / IMG_W
    scale_y = frame_h / IMG_H

    if len(keep) > 0:
        boxes = decode_boxes(loc[keep], anchors[keep])
        scores = face_scores[keep]
        keep_nms = nms(boxes, scores, NMS_THRESH)
        for i in keep_nms:
            x1, y1, x2, y2 = boxes[i].astype(np.int32)
            x1 = int(x1 * scale_x)
            x2 = int(x2 * scale_x)
            y1 = int(y1 * scale_y)
            y2 = int(y2 * scale_y)
            cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 0, 255), 1)
            cv2.putText(frame, f"{scores[i]:.2f}", (x1, max(y1 - 5, 0)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)

    # === Affichage FPS dans un rectangle noir ===
    fps = 1.0 / (time.time() - start_time)
    fps_text = f"FPS: {fps:.1f}"
    fps_pos = (500, 25)

    # Calcul taille du texte et rectangle
    font = cv2.FONT_HERSHEY_SIMPLEX
    font_scale = 0.7
    thickness = 1
    text_color = (255, 0, 0)  # Bleu BGR

    (text_w, text_h), baseline = cv2.getTextSize(fps_text, font, font_scale, thickness)
    x, y = fps_pos

    # Rectangle : laisse une marge autour du texte
    margin = 5
    top_left = (x - margin, y - text_h - margin)
    bottom_right = (x + text_w + margin, y + baseline + margin)

    # Dessine rectangle noir (épaisseur 1)
    cv2.rectangle(frame, top_left, bottom_right, (0, 0, 0), 1)

    # Texte FPS, par-dessus le rectangle
    cv2.putText(frame, fps_text, fps_pos, font, font_scale, text_color, thickness, lineType=cv2.LINE_AA)

    cv2.imshow(WINDOW_NAME, frame)
    key = cv2.waitKey(1) & 0xFF
    if key == 27:
        break

cap.release()
cv2.destroyAllWindows()
print(f"[INFO] Terminé. {frame_id} frames traitées.")