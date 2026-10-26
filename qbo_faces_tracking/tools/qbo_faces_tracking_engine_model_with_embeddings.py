#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Détection de visages UltraFace RFB-320 TensorRT (.engine) sur vidéo locale.
Ajout : Recherche embeddings visage dans base .npz (embeddings + noms).
Ultra optimisé Jetson Orin NX.
VERSION SIMPLIFIÉE : Sans InsightFace, utilise des features d'image personnalisées
"""

# =========================================
# === IMPORTS ET LIBRAIRIES PRINCIPALES ===
# =========================================
import cv2
import numpy as np
import tensorrt as trt
import pycuda.driver as cuda
import pycuda.autoinit
import time
from math import ceil

# =======================================
# === PARAMÈTRES GLOBAUX DU SCRIPT   ====
# =======================================
VIDEO_PATH = "/home/nvidia/Downloads/VID_20250719_114809795_2_640x480.mp4"
ENGINE_PATH = "/home/nvidia/qbo_ws/src/qbo_faces_tracking/tools/version-RFB-320.engine"
IMG_W, IMG_H = 320, 240
CONF_THRESH = 0.85
NMS_THRESH = 0.3
VARIANCES = [0.1, 0.2]
WINDOW_NAME = "UltraFace RFB-320 - TRT"
EMBEDDING_PATH = "/home/nvidia/Documents/face_embeddings.npz"  # Base embeddings

# ==================================
# === GÉNÉRATION DES ANCRES     ====
# ==================================
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

# ==================================
# === PRÉTRAITEMENT DE L'IMAGE   ===
# ==================================
def preprocess(img):
    img = cv2.resize(img, (IMG_W, IMG_H))
    img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
    img = (img.astype(np.float32) - 127.0) / 128.0
    img = np.transpose(img, (2, 0, 1))
    return np.expand_dims(img, axis=0).astype(np.float32)

# ====================================
# === DÉCODAGE DES BBOX PRÉDITES   ===
# ====================================
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

# ========================================
# === FONCTIONS NMS (SUPPRESSION BBOX) ===
# ========================================
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

# ==============================================
# === INITIALISATION DU MOTEUR TENSORRT     ====
# ==============================================
print("[INFO] Chargement de l'engine TensorRT...")
TRT_LOGGER = trt.Logger(trt.Logger.WARNING)
with open(ENGINE_PATH, 'rb') as f:
    engine_data = f.read()
runtime = trt.Runtime(TRT_LOGGER)
engine = runtime.deserialize_cuda_engine(engine_data)
context = engine.create_execution_context()

binding_names = [engine.get_tensor_name(i) for i in range(engine.num_io_tensors)]
binding_shapes = [engine.get_tensor_shape(name) for name in binding_names]
is_input = [engine.get_tensor_mode(name) == trt.TensorIOMode.INPUT for name in binding_names]

print("[DEBUG] Liste des bindings (nom, shape, INPUT?):")
for i, name in enumerate(binding_names):
    print(f"  {i}: {name} | {binding_shapes[i]} | {'INPUT' if is_input[i] else 'OUTPUT'}")

input_name = None
conf_name = None
loc_name = None
for i, name in enumerate(binding_names):
    shp = binding_shapes[i]
    if is_input[i]:
        input_name = name
    else:
        if shp[-1] == 2:
            conf_name = name
        elif shp[-1] == 4:
            loc_name = name
if not input_name or not conf_name or not loc_name:
    raise RuntimeError("Impossible d'inférer les noms des tensors dans l'engine !")

print(f"[INFO] Tensors utilisés : input={input_name}, conf={conf_name}, loc={loc_name}")

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

output_shapes = [conf_shape, loc_shape]

# =====================================
# === FONCTION D'INFÉRENCE GPU    =====
# =====================================
def infer_trt(img):
    img_input = preprocess(img).flatten().astype(np.float32)
    np.copyto(input_host, img_input.ravel())
    cuda.memcpy_htod_async(input_dev, input_host, stream)
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

# =========================================
# === CALCUL ET STOCKAGE DES ANCRES    ===
# =========================================
anchors = generate_anchors_ultraface_4420(IMG_W, IMG_H)

# =========================================
# === CHARGEMENT EMBEDDINGS DE RÉF.    ===
# =========================================
def load_known_embeddings(path=EMBEDDING_PATH):
    """
    Charge un fichier .npz contenant deux arrays : 'embeddings' (N,512) et 'names' (N,).
    Reconstitue un dict {nom: embedding}.
    """
    data = np.load(path, allow_pickle=True)
    embs = data["embeddings"]   # shape (N, 512)
    names = data["names"]       # shape (N,)
    print(f"[INFO] {len(names)} embeddings connus chargés: {list(names)}")
    return {str(names[i]): embs[i] for i in range(len(names))}

known_embeddings = load_known_embeddings()

# =========================================
# === COMPARAISON D'EMBEDDINGS       =====
# =========================================
def compare_embeddings(face_emb, embeddings_db, threshold=0.6):
    """
    Compare l'embedding du visage courant à la base connue.
    Utilise la distance cosinus pour la comparaison.
    """
    if face_emb is None:
        return "Unknown", float("inf")
        
    min_dist = float("inf")
    name = "Unknown"
    
    for k, v in embeddings_db.items():
        if v.shape != face_emb.shape:
            continue
        
        # Distance cosinus (1 - similarité cosinus)
        cos_sim = np.dot(face_emb, v) / (np.linalg.norm(face_emb) * np.linalg.norm(v) + 1e-6)
        dist = 1 - cos_sim
        
        if dist < min_dist:
            min_dist = dist
            name = k
    
    if min_dist > threshold:
        name = "Unknown"
        
    return name, min_dist

# ==========================================
# === EXTRACTION EMBEDDING PERSONNALISÉ ====
# ==========================================
def get_face_embedding(face_img):
    """
    Extrait un embedding personnalisé basé sur les caractéristiques de l'image du visage.
    Cette version améliore les features pour une meilleure discrimination.
    """
    try:
        # Redimensionner à une taille fixe
        face_resized = cv2.resize(face_img, (112, 112))
        
        # Convertir en plusieurs espaces couleur
        gray = cv2.cvtColor(face_resized, cv2.COLOR_BGR2GRAY)
        hsv = cv2.cvtColor(face_resized, cv2.COLOR_BGR2HSV)
        
        embedding = np.zeros(512, dtype=np.float32)
        idx = 0
        
        # === FEATURES STATISTIQUES GLOBALES ===
        embedding[idx] = np.mean(gray) / 255.0; idx += 1
        embedding[idx] = np.std(gray) / 255.0; idx += 1
        embedding[idx] = np.median(gray) / 255.0; idx += 1
        embedding[idx] = np.var(gray) / (255.0**2); idx += 1
        
        # === HISTOGRAMMES ===
        # Histogramme luminosité (16 bins)
        hist_gray = np.histogram(gray, bins=16, range=(0, 256))[0] / (112*112)
        embedding[idx:idx+16] = hist_gray; idx += 16
        
        # Histogramme couleur HSV (8 bins par canal)
        hist_h = np.histogram(hsv[:,:,0], bins=8, range=(0, 180))[0] / (112*112)
        hist_s = np.histogram(hsv[:,:,1], bins=8, range=(0, 256))[0] / (112*112)
        hist_v = np.histogram(hsv[:,:,2], bins=8, range=(0, 256))[0] / (112*112)
        embedding[idx:idx+8] = hist_h; idx += 8
        embedding[idx:idx+8] = hist_s; idx += 8
        embedding[idx:idx+8] = hist_v; idx += 8
        
        # === FEATURES DE TEXTURE (GRADIENTS) ===
        # Gradients Sobel
        grad_x = cv2.Sobel(gray, cv2.CV_64F, 1, 0, ksize=3)
        grad_y = cv2.Sobel(gray, cv2.CV_64F, 0, 1, ksize=3)
        magnitude = np.sqrt(grad_x**2 + grad_y**2)
        
        embedding[idx] = np.mean(np.abs(grad_x)) / 255.0; idx += 1
        embedding[idx] = np.mean(np.abs(grad_y)) / 255.0; idx += 1
        embedding[idx] = np.mean(magnitude) / 255.0; idx += 1
        embedding[idx] = np.std(magnitude) / 255.0; idx += 1
        
        # === LBP (Local Binary Patterns) simplifiés ===
        def simple_lbp_feature(img, radius=1):
            h, w = img.shape
            lbp_hist = np.zeros(256)
            for i in range(radius, h-radius):
                for j in range(radius, w-radius):
                    center = img[i, j]
                    code = 0
                    # 8 voisins
                    neighbors = [
                        img[i-radius, j-radius], img[i-radius, j], img[i-radius, j+radius],
                        img[i, j+radius], img[i+radius, j+radius], img[i+radius, j],
                        img[i+radius, j-radius], img[i, j-radius]
                    ]
                    for k, neighbor in enumerate(neighbors):
                        if neighbor >= center:
                            code |= (1 << k)
                    lbp_hist[code] += 1
            return lbp_hist / np.sum(lbp_hist)
        
        lbp_features = simple_lbp_feature(gray)[:32]  # Prendre les 32 premiers bins
        embedding[idx:idx+32] = lbp_features; idx += 32
        
        # === FEATURES RÉGIONALES ===
        # Diviser l'image en 4 quadrants et extraire des stats pour chaque
        h, w = gray.shape
        quadrants = [
            gray[:h//2, :w//2],    # Top-left
            gray[:h//2, w//2:],    # Top-right
            gray[h//2:, :w//2],    # Bottom-left
            gray[h//2:, w//2:]     # Bottom-right
        ]
        
        for quad in quadrants:
            embedding[idx] = np.mean(quad) / 255.0; idx += 1
            embedding[idx] = np.std(quad) / 255.0; idx += 1
            embedding[idx] = np.median(quad) / 255.0; idx += 1
        
        # === FEATURES FRÉQUENTIELLES (DCT) ===
        # DCT sur l'image pour capturer les patterns de fréquence
        gray_float = np.float32(gray)
        dct = cv2.dct(gray_float)
        # Prendre les coefficients DCT les plus significatifs (basses fréquences)
        dct_features = dct[:16, :16].flatten()[:64]  # 64 premiers coefficients
        dct_features = dct_features / (np.max(np.abs(dct_features)) + 1e-6)  # Normaliser
        embedding[idx:idx+64] = dct_features; idx += 64
        
        # === REMPLIR LE RESTE ===
        # Pour atteindre 512 dimensions, créer des combinaisons des features existantes
        while idx < 512:
            remaining = 512 - idx
            base_features = embedding[:min(idx, remaining)]
            if len(base_features) > 0:
                # Créer des variations avec des transformations non-linéaires
                variations = np.tanh(base_features * 2.0) * 0.1
                n_to_add = min(len(variations), remaining)
                embedding[idx:idx+n_to_add] = variations[:n_to_add]
                idx += n_to_add
            else:
                break
        
        # === NORMALISATION FINALE ===
        norm = np.linalg.norm(embedding)
        if norm > 0:
            embedding = embedding / norm
        
        return embedding
        
    except Exception as e:
        print(f"[ERROR] Erreur extraction embedding: {e}")
        # Retourner un embedding aléatoire normalisé en cas d'erreur
        random_emb = np.random.normal(0, 0.1, 512).astype(np.float32)
        return random_emb / np.linalg.norm(random_emb)

# =========================================
# === BOUCLE PRINCIPALE VIDÉO + AFFICHAGE =
# =========================================
cap = cv2.VideoCapture(VIDEO_PATH)
if not cap.isOpened():
    print(f"[ERREUR] Impossible d'ouvrir la vidéo : {VIDEO_PATH}")
    exit(1)

cv2.namedWindow(WINDOW_NAME, cv2.WINDOW_NORMAL)
cv2.resizeWindow(WINDOW_NAME, 800, 600)
frame_id = 0

print("[INFO] 🚀 Démarrage de la reconnaissance faciale (version personnalisée)...")
print(f"[INFO] Personnes connues: {list(known_embeddings.keys())}")

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
            x1_img = int(x1 * scale_x)
            x2_img = int(x2 * scale_x)
            y1_img = int(y1 * scale_y)
            y2_img = int(y2 * scale_y)

            # === Crop visage pour embedding ===
            face_crop = frame[max(y1_img, 0):min(y2_img, frame_h), max(x1_img, 0):min(x2_img, frame_w)]
            if face_crop.size == 0 or face_crop.shape[0] < 10 or face_crop.shape[1] < 10:
                continue

            # Extraction d'embedding pour ce visage
            face_emb = get_face_embedding(face_crop)

            # Recherche dans la base d'embeddings connus
            name, dist = compare_embeddings(face_emb, known_embeddings)

            # Couleur du rectangle selon la reconnaissance
            color = (0, 255, 0) if name != "Unknown" else (0, 0, 255)  # Vert si connu, Rouge si inconnu
            
            # Dessin rectangle + nom (+distance)
            cv2.rectangle(frame, (x1_img, y1_img), (x2_img, y2_img), color, 2)
            label = f"{name} ({dist:.2f})" if name != "Unknown" else "Unknown"
            
            # Fond pour le texte pour meilleure lisibilité
            font = cv2.FONT_HERSHEY_SIMPLEX
            font_scale = 0.6
            thickness = 2
            (text_w, text_h), _ = cv2.getTextSize(label, font, font_scale, thickness)
            
            # Rectangle de fond pour le texte
            cv2.rectangle(frame, (x1_img, y1_img - text_h - 10), (x1_img + text_w, y1_img), (0, 0, 0), -1)
            cv2.putText(frame, label, (x1_img, y1_img - 5), font, font_scale, (255, 255, 255), thickness)

    # === Affichage FPS ===
    fps = 1.0 / (time.time() - start_time)
    fps_text = f"FPS: {fps:.1f}"
    fps_pos = (500, 25)
    font = cv2.FONT_HERSHEY_SIMPLEX
    font_scale = 0.7
    thickness = 1
    text_color = (255, 0, 0)
    (text_w, text_h), baseline = cv2.getTextSize(fps_text, font, font_scale, thickness)
    x, y = fps_pos
    margin = 5
    top_left = (x - margin, y - text_h - margin)
    bottom_right = (x + text_w + margin, y + baseline + margin)
    cv2.rectangle(frame, top_left, bottom_right, (0, 0, 0), -1)
    cv2.putText(frame, fps_text, fps_pos, font, font_scale, text_color, thickness, lineType=cv2.LINE_AA)

    cv2.imshow(WINDOW_NAME, frame)
    key = cv2.waitKey(1) & 0xFF
    if key == 27:  # ESC pour quitter
        break

cap.release()
cv2.destroyAllWindows()
print(f"[INFO] Terminé. {frame_id} frames traitées.")