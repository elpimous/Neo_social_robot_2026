#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Reconnaissance faciale UltraFace + ArcFace avec TensorRT (Jetson Orin NX).
Version sans affichage pour performance maximale.
Affiche uniquement le FPS moyen à la fin.
"""

import cv2
import numpy as np
import tensorrt as trt
import pycuda.driver as cuda
import pycuda.autoinit
import time
import os
from math import ceil

# === PARAMÈTRES ===
VIDEO_PATH = "/home/nvidia/Downloads/VID_20250719_114809795_2_640x480.mp4"
ENGINE_PATH = "/home/nvidia/qbo_ws/src/qbo_faces_tracking/tools/version-RFB-320.engine"
ARCFACE_ENGINE_PATH = "/home/nvidia/qbo_ws/src/qbo_faces_tracking/models/w600k_mbf.engine"
EMBEDDING_PATH = "/home/nvidia/qbo_ws/src/qbo_faces_tracking/face_embeddings/face_embeddings_mobile.npz"
IMG_W, IMG_H = 320, 240
CONF_THRESH = 0.99
NMS_THRESH = 0.3
VARIANCES = [0.1, 0.2]

# === ANCRES ULTRAFACE ===
def generate_anchors_ultraface_4420(img_w=320, img_h=240):
    min_sizes = [[10, 16, 24], [32, 48], [64, 96], [128, 192, 256]]
    steps = [8, 16, 32, 64]
    feature_maps = [[ceil(img_h / s), ceil(img_w / s)] for s in steps]
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
    return np.array(anchors, dtype=np.float32)

# === PRÉTRAITEMENT ULTRAFACE ===
def preprocess(img):
    img = cv2.resize(img, (IMG_W, IMG_H))
    img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
    img = (img.astype(np.float32) - 127.0) / 128.0
    img = np.transpose(img, (2, 0, 1))
    return np.expand_dims(img, axis=0).astype(np.float32)

# === DÉCODAGE BBOX ===
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

# === NMS ===
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

# === EMBEDDINGS BASE ===
def load_known_embeddings(path):
    data = np.load(path, allow_pickle=True)
    return {str(data["names"][i]): data["embeddings"][i] for i in range(len(data["names"]))}

known_embeddings = load_known_embeddings(EMBEDDING_PATH)

# === ULTRAFACE TRT ENGINE ===
TRT_LOGGER = trt.Logger(trt.Logger.WARNING)
with open(ENGINE_PATH, 'rb') as f:
    engine = trt.Runtime(TRT_LOGGER).deserialize_cuda_engine(f.read())
context = engine.create_execution_context()

input_name = [n for n in engine if engine.get_tensor_mode(n) == trt.TensorIOMode.INPUT][0]
conf_name = [n for n in engine if engine.get_tensor_shape(n)[-1] == 2][0]
loc_name = [n for n in engine if engine.get_tensor_shape(n)[-1] == 4][0]

input_shape = engine.get_tensor_shape(input_name)
conf_shape = engine.get_tensor_shape(conf_name)
loc_shape = engine.get_tensor_shape(loc_name)

input_host = cuda.pagelocked_empty([int(np.prod(input_shape))], dtype=np.float32)
input_dev = cuda.mem_alloc(input_host.nbytes)
output_host_conf = cuda.pagelocked_empty([int(np.prod(conf_shape))], dtype=np.float32)
output_dev_conf = cuda.mem_alloc(output_host_conf.nbytes)
output_host_loc = cuda.pagelocked_empty([int(np.prod(loc_shape))], dtype=np.float32)
output_dev_loc = cuda.mem_alloc(output_host_loc.nbytes)
stream = cuda.Stream()

def infer_trt(img):
    np.copyto(input_host, preprocess(img).flatten())
    cuda.memcpy_htod_async(input_dev, input_host, stream)
    context.set_tensor_address(input_name, int(input_dev))
    context.set_tensor_address(conf_name, int(output_dev_conf))
    context.set_tensor_address(loc_name, int(output_dev_loc))
    context.execute_async_v3(stream_handle=stream.handle)
    cuda.memcpy_dtoh_async(output_host_conf, output_dev_conf, stream)
    cuda.memcpy_dtoh_async(output_host_loc, output_dev_loc, stream)
    stream.synchronize()
    return output_host_conf.reshape(conf_shape), output_host_loc.reshape(loc_shape)

anchors = generate_anchors_ultraface_4420(IMG_W, IMG_H)

# === ARCFACE TRT ENGINE ===
arcface_logger = trt.Logger(trt.Logger.WARNING)
with open(ARCFACE_ENGINE_PATH, 'rb') as f:
    arcface_engine = trt.Runtime(arcface_logger).deserialize_cuda_engine(f.read())
arcface_context = arcface_engine.create_execution_context()

arcface_input_name = [n for n in arcface_engine if arcface_engine.get_tensor_mode(n) == trt.TensorIOMode.INPUT][0]
arcface_output_name = [n for n in arcface_engine if arcface_engine.get_tensor_mode(n) == trt.TensorIOMode.OUTPUT][0]
arcface_input_shape = arcface_engine.get_tensor_shape(arcface_input_name)
arcface_output_shape = arcface_engine.get_tensor_shape(arcface_output_name)

arcface_input_host = cuda.pagelocked_empty([int(np.prod(arcface_input_shape))], dtype=np.float32)
arcface_input_dev = cuda.mem_alloc(arcface_input_host.nbytes)
arcface_output_host = cuda.pagelocked_empty([int(np.prod(arcface_output_shape))], dtype=np.float32)
arcface_output_dev = cuda.mem_alloc(arcface_output_host.nbytes)
arcface_stream = cuda.Stream()

def preprocess_arcface(face_img):
    img = cv2.resize(face_img, (112, 112))
    img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
    img = (img.astype(np.float32) - 127.5) / 128.0
    img = np.transpose(img, (2, 0, 1))
    return np.expand_dims(img, axis=0).astype(np.float32)

def extract_embedding_arcface(face_img):
    face_input = preprocess_arcface(face_img).flatten()
    np.copyto(arcface_input_host, face_input)
    cuda.memcpy_htod_async(arcface_input_dev, arcface_input_host, arcface_stream)
    arcface_context.set_tensor_address(arcface_input_name, int(arcface_input_dev))
    arcface_context.set_tensor_address(arcface_output_name, int(arcface_output_dev))
    arcface_context.execute_async_v3(stream_handle=arcface_stream.handle)
    cuda.memcpy_dtoh_async(arcface_output_host, arcface_output_dev, arcface_stream)
    arcface_stream.synchronize()
    emb = arcface_output_host.reshape(arcface_output_shape)
    norm = np.linalg.norm(emb)
    return emb / (norm + 1e-10)

def compare_embeddings(face_emb, embeddings_db, threshold=1.3):
    min_dist, name = float('inf'), "Unknown"
    for k, v in embeddings_db.items():
        if v.shape == face_emb.shape:
            dist = 2 - 2 * np.dot(face_emb, v)
            if dist < min_dist:
                min_dist, name = dist, k
    return (name if min_dist < threshold else "Unknown", min_dist)

# === BOUCLE PRINCIPALE (SANS AFFICHAGE) ===
cap = cv2.VideoCapture(VIDEO_PATH)
if not cap.isOpened():
    print(f"[ERREUR] Impossible d'ouvrir la vidéo : {VIDEO_PATH}")
    exit(1)

frame_id = 0
start_global = time.time()

while True:
    ret, frame = cap.read()
    if not ret:
        break

    frame_id += 1
    frame_h, frame_w = frame.shape[:2]
    scale_x = frame_w / IMG_W
    scale_y = frame_h / IMG_H

    conf, loc = infer_trt(frame)
    scores = conf[0][:, 1]
    boxes = decode_boxes(loc[0], anchors)
    indices = np.where(scores > CONF_THRESH)[0]

    if len(indices):
        boxes = boxes[indices]
        scores = scores[indices]
        for i in nms(boxes, scores, NMS_THRESH):
            x1, y1, x2, y2 = boxes[i]
            x1_img = max(0, min(frame_w, int(x1 * scale_x)))
            y1_img = max(0, min(frame_h, int(y1 * scale_y)))
            x2_img = max(0, min(frame_w, int(x2 * scale_x)))
            y2_img = max(0, min(frame_h, int(y2 * scale_y)))
            face_crop = frame[y1_img:y2_img, x1_img:x2_img]
            if face_crop.size == 0:
                continue
            face_emb = extract_embedding_arcface(face_crop)
            compare_embeddings(face_emb, known_embeddings)

# === Fin de vidéo : calcul et affichage du FPS global ===
total_time = time.time() - start_global
fps_moyen = frame_id / total_time
print(f"[INFO] Vidéo traitée : {frame_id} frames en {total_time:.2f} sec")
print(f"[INFO] FPS moyen : {fps_moyen:.2f}")
