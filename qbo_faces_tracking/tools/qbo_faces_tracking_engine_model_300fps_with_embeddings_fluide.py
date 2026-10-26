import cv2
import numpy as np
import tensorrt as trt
import pycuda.driver as cuda
import pycuda.autoinit
import time
from math import ceil

# ... (toutes tes définitions précédentes inchangées)

# Ajout : Gestion d'identités et de tracking léger par IoU
class TrackedBox:
    def __init__(self, box, track_id):
        self.box = np.array(box, dtype=np.float32)
        self.id = track_id
        self.age = 0           # Nombre de frames depuis la dernière apparition
        self.hits = 1          # Nombre de frames où la box a été matchée

    def update(self, box, alpha=0.7):
        # Lissage exponentiel sur la bbox pour éviter les "sauts" brusques
        self.box = alpha * self.box + (1 - alpha) * np.array(box, dtype=np.float32)
        self.hits += 1
        self.age = 0

def iou_single(box1, box2):
    # box: [x1, y1, x2, y2]
    xA = max(box1[0], box2[0])
    yA = max(box1[1], box2[1])
    xB = min(box1[2], box2[2])
    yB = min(box1[3], box2[3])
    interArea = max(0, xB - xA) * max(0, yB - yA)
    area1 = (box1[2] - box1[0]) * (box1[3] - box1[1])
    area2 = (box2[2] - box2[0]) * (box2[3] - box2[1])
    return interArea / (area1 + area2 - interArea + 1e-6)

def match_tracks(tracked_boxes, detections, iou_thresh=0.3):
    """
    Pour chaque détection, cherche la tracked_box la plus proche (IoU > threshold).
    Retourne:
        - matches: liste de tuples (det_idx, track_idx)
        - unmatched_dets: indices des detections non associées
        - unmatched_tracks: indices des tracks non associées
    """
    if len(tracked_boxes) == 0 or len(detections) == 0:
        return [], list(range(len(detections))), list(range(len(tracked_boxes)))
    iou_matrix = np.zeros((len(detections), len(tracked_boxes)), dtype=np.float32)
    for d, det in enumerate(detections):
        for t, trk in enumerate(tracked_boxes):
            iou_matrix[d, t] = iou_single(det, trk.box)
    matches = []
    det_matched, trk_matched = set(), set()
    while True:
        if iou_matrix.size == 0 or np.max(iou_matrix) < iou_thresh:
            break
        d, t = np.unravel_index(np.argmax(iou_matrix), iou_matrix.shape)
        matches.append((d, t))
        det_matched.add(d)
        trk_matched.add(t)
        iou_matrix[d, :] = -1
        iou_matrix[:, t] = -1
    unmatched_dets = [d for d in range(len(detections)) if d not in det_matched]
    unmatched_tracks = [t for t in range(len(tracked_boxes)) if t not in trk_matched]
    return matches, unmatched_dets, unmatched_tracks

# ===== Boucle principale avec tracking "fluide" =====
cap = cv2.VideoCapture(VIDEO_PATH)
if not cap.isOpened():
    print(f"[ERREUR] Impossible d'ouvrir la vidéo : {VIDEO_PATH}")
    exit(1)

cv2.namedWindow(WINDOW_NAME, cv2.WINDOW_NORMAL)
cv2.resizeWindow(WINDOW_NAME, 800, 600)
frame_id = 0

tracked_faces = []
next_track_id = 1

while True:
    ret, frame = cap.read()
    if not ret:
        break
    frame_id += 1
    start = time.time()

    frame_h, frame_w = frame.shape[:2]
    scale_x = frame_w / IMG_W
    scale_y = frame_h / IMG_H

    conf, loc = infer_trt(frame)
    scores = conf[0][:, 1]
    boxes = decode_boxes(loc[0], anchors)
    indices = np.where(scores > CONF_THRESH)[0]

    det_boxes = []
    if len(indices):
        boxes = boxes[indices]
        scores = scores[indices]
        for i in nms(boxes, scores, NMS_THRESH):
            x1, y1, x2, y2 = boxes[i]
            # Upscale vers la frame originale
            x1_img = max(0, min(frame.shape[1], int(x1 * scale_x)))
            y1_img = max(0, min(frame.shape[0], int(y1 * scale_y)))
            x2_img = max(0, min(frame.shape[1], int(x2 * scale_x)))
            y2_img = max(0, min(frame.shape[0], int(y2 * scale_y)))
            # Ignore crops trop petits
            if (x2_img - x1_img < 10) or (y2_img - y1_img < 10):
                continue
            det_boxes.append([x1_img, y1_img, x2_img, y2_img])

    # --- Association tracking ---
    matches, unmatched_dets, unmatched_tracks = match_tracks(tracked_faces, det_boxes, iou_thresh=0.4)
    # Update matched tracks
    for det_idx, track_idx in matches:
        tracked_faces[track_idx].update(det_boxes[det_idx])
    # Add new tracks
    for det_idx in unmatched_dets:
        tracked_faces.append(TrackedBox(det_boxes[det_idx], next_track_id))
        next_track_id += 1
    # Vieillit les tracks non matchées
    for track_idx in unmatched_tracks:
        tracked_faces[track_idx].age += 1
    # Supprime les tracks trop vieux (plus vus depuis 10 frames)
    tracked_faces = [trk for trk in tracked_faces if trk.age <= 10]

    # --- Affichage + Embedding ---
    for trk in tracked_faces:
        x1, y1, x2, y2 = map(int, trk.box)
        face_crop = frame[y1:y2, x1:x2]
        if face_crop.size == 0 or face_crop.shape[0] < 10 or face_crop.shape[1] < 10:
            continue
        face_emb = extract_embedding_arcface(face_crop)
        if np.any(np.isnan(face_emb)) or np.any(np.isinf(face_emb)):
            continue
        name, dist = compare_embeddings(face_emb, known_embeddings)
        label = f"#{trk.id}:{name} ({dist:.2f})"
        color = (0, 255, 0) if name != "Unknown" else (0, 128, 255)
        cv2.rectangle(frame, (x1, y1), (x2, y2), color, 2)
        cv2.putText(frame, label, (x1, max(0, y1 - 10)), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)

    fps = 1.0 / (time.time() - start)
    cv2.putText(frame, f"FPS: {fps:.1f}", (5, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 0, 0), 2)
    cv2.imshow(WINDOW_NAME, frame)
    if cv2.waitKey(1) & 0xFF == 27:
        break

cap.release()
cv2.destroyAllWindows()
print(f"[INFO] Terminé. {frame_id} frames traitées.")
