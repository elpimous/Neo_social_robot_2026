#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Node ROS2 : qbo_face_tracking_node.py
- Détection visage (UltraFace TRT) à 30fps pour ROI fluide
- Reconnaissance (ArcFace TRT) à 2fps max pour économiser GPU
- Publication /face_roi, /is_recognized, /face_tracking_debug
- Nez RGB : vert = reconnu, rouge = inconnu, éteint = absent

Auteur : Vincent FOUCAULT 2025.
"""

import rclpy
from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory
import cv2
import numpy as np
import tensorrt as trt
import pycuda.driver as cuda
import pycuda.autoinit
from sensor_msgs.msg import Image, RegionOfInterest
from std_msgs.msg import String
from qbo_msgs.msg import Nose
from cv_bridge import CvBridge
import time
import os
from math import ceil
from functools import lru_cache

# ================== PARAMÈTRES =====================
package_share_dir   = get_package_share_directory("qbo_faces_tracking")
ENGINE_PATH         = os.path.join(package_share_dir, "models", "version-RFB-320_fp16.engine")
ARCFACE_ENGINE_PATH = os.path.join(package_share_dir, "models", "w600k_mbf_fp16.engine")
EMBEDDING_PATH      = os.path.join(package_share_dir, "face_embeddings", "face_embeddings_average.npz")

IMG_W, IMG_H      = 320, 240
CONF_THRESH       = 0.90
NMS_THRESH        = 0.1
VARIANCES         = [0.1, 0.2]
RECOGNITION_FPS   = 2.0
DETECTION_TIMEOUT = 0.5   # secondes avant de considérer le visage disparu
RECO_THRESHOLD    = 1.0   # distance cosinus max pour valider une reconnaissance


# ============================================================
#  COMMENT AJOUTER UNE NOUVELLE IDENTITÉ DANS LA BASE
# ============================================================
#
#  Principe :
#    1. Capturer N crops de visage sur quelques secondes
#    2. Extraire l'embedding ArcFace (512-dim) pour chaque crop
#    3. Faire la moyenne de tous les embeddings, normaliser
#    4. Sauvegarder dans le .npz via add_embedding_to_database()
#
#  Utilisation standalone (script séparé, hors node ROS2) :
#  ──────────────────────────────────────────────────────────
#
#    import pycuda.autoinit
#    import pycuda.driver as cuda
#    import tensorrt as trt
#    import numpy as np, cv2, time
#
#    # Initialiser ArcFace TRT
#    af_engine, af_ctx = init_trt_engine(ARCFACE_ENGINE_PATH)
#    af_input_name     = next(n for n in af_engine if af_engine.get_tensor_mode(n) == trt.TensorIOMode.INPUT)
#    af_output_name    = next(n for n in af_engine if af_engine.get_tensor_mode(n) == trt.TensorIOMode.OUTPUT)
#    af_in_shape       = af_engine.get_tensor_shape(af_input_name)
#    af_out_shape      = af_engine.get_tensor_shape(af_output_name)
#    af_in_host        = cuda.pagelocked_empty(int(np.prod(af_in_shape)),  np.float32)
#    af_in_dev         = cuda.mem_alloc(af_in_host.nbytes)
#    af_out_host       = cuda.pagelocked_empty(int(np.prod(af_out_shape)), np.float32)
#    af_out_dev        = cuda.mem_alloc(af_out_host.nbytes)
#    af_stream         = cuda.Stream()
#
#    # Initialiser aussi UltraFace pour détecter le crop dans chaque frame
#    # (voir _detect() dans le node pour le détail)
#
#    # Boucle de capture (~25 embeddings en 4 secondes à ~6fps)
#    TARGET   = 25
#    INTERVAL = 0.15        # secondes entre deux extractions
#    embeddings = []
#    last_t     = 0
#    cap        = cv2.VideoCapture(0)
#
#    while len(embeddings) < TARGET:
#        ret, frame = cap.read()
#        if not ret:
#            continue
#        now = time.monotonic()
#        if now - last_t < INTERVAL:
#            continue
#        last_t = now
#
#        det = _detect(frame, ...)          # utiliser la fonction _detect() du node
#        if det is None:
#            continue
#
#        emb = extract_embedding_arcface(
#            det['crop'],
#            af_ctx, af_input_name, af_output_name,
#            af_in_host, af_in_dev, af_out_host, af_out_dev, af_stream
#        )
#        if emb is not None:
#            embeddings.append(emb)
#            print(f"  Embedding {len(embeddings)}/{TARGET}")
#
#    cap.release()
#
#    # Calcul embedding moyen + normalisation
#    mean_emb = np.mean(np.array(embeddings), axis=0)
#    norm     = np.linalg.norm(mean_emb)
#    if norm > 1e-10:
#        mean_emb /= norm
#
#    # Chargement base existante, ajout, sauvegarde
#    known = load_embeddings(EMBEDDING_PATH)          # → dict {nom: emb}
#    known["NouvellePersonne"] = mean_emb             # ← changer le nom ici
#    names          = list(known.keys())
#    embeddings_arr = np.array([known[n] for n in names])
#    np.savez_compressed(EMBEDDING_PATH, names=names, embeddings=embeddings_arr)
#    print(f"Base sauvegardée : {len(names)} identité(s)")
#
#
#  Utilisation depuis un thread externe pendant que le node tourne :
#  ─────────────────────────────────────────────────────────────────
#
#    # Le node expose self._current_crop (à ajouter si besoin)
#    # et add_embedding_to_database() pour la sauvegarde finale.
#
#    embeddings = []
#    TARGET     = 25
#    INTERVAL   = 0.15
#    last_t     = 0
#
#    while len(embeddings) < TARGET:
#        now = time.monotonic()
#        if now - last_t < INTERVAL:
#            time.sleep(0.02)
#            continue
#        last_t = now
#
#        if node._face_present:                       # visage actuellement détecté
#            emb = extract_embedding_arcface(
#                node._current_crop,                  # crop de la dernière détection
#                node.af_ctx,
#                node._af_input_name, node._af_output_name,
#                node._af_in_host,    node._af_in_dev,
#                node._af_out_host,   node._af_out_dev,
#                node._af_stream
#            )
#            if emb is not None:
#                embeddings.append(emb)
#
#    mean_emb = np.mean(np.array(embeddings), axis=0)
#    norm     = np.linalg.norm(mean_emb)
#    if norm > 1e-10:
#        mean_emb /= norm
#    node.add_embedding_to_database("NouvellePersonne", mean_emb)
#
# ============================================================


# =============== UTILITAIRES TRT ================

@lru_cache(maxsize=1)
def generate_anchors(img_w=320, img_h=240):
    min_sizes    = [[10, 16, 24], [32, 48], [64, 96], [128, 192, 256]]
    steps        = [8, 16, 32, 64]
    feature_maps = [[ceil(img_h / s), ceil(img_w / s)] for s in steps]
    anchors = []
    for k, f in enumerate(feature_maps):
        for i in range(f[0]):
            for j in range(f[1]):
                for ms in min_sizes[k]:
                    anchors.append([
                        (j + 0.5) * steps[k] / img_w,
                        (i + 0.5) * steps[k] / img_h,
                        ms / img_w,
                        ms / img_h,
                    ])
    return np.array(anchors, dtype=np.float32)


def preprocess_ultraface(img):
    img = cv2.resize(img, (IMG_W, IMG_H), interpolation=cv2.INTER_AREA)
    img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
    img = (img.astype(np.float32) - 127.0) / 128.0
    return np.expand_dims(np.transpose(img, (2, 0, 1)), 0).astype(np.float32)


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


def nms(boxes, scores, threshold=0.3):
    if len(boxes) == 0:
        return []
    idxs = np.argsort(-scores)
    keep = []
    while len(idxs) > 0:
        i = idxs[0]
        keep.append(i)
        if len(idxs) == 1:
            break
        inter = (
            np.maximum(0, np.minimum(boxes[idxs[1:], 2], boxes[i, 2]) - np.maximum(boxes[idxs[1:], 0], boxes[i, 0])) *
            np.maximum(0, np.minimum(boxes[idxs[1:], 3], boxes[i, 3]) - np.maximum(boxes[idxs[1:], 1], boxes[i, 1]))
        )
        area1    = (boxes[i, 2] - boxes[i, 0]) * (boxes[i, 3] - boxes[i, 1])
        area2    = (boxes[idxs[1:], 2] - boxes[idxs[1:], 0]) * (boxes[idxs[1:], 3] - boxes[idxs[1:], 1])
        iou_vals = inter / (area1 + area2 - inter + 1e-6)
        idxs     = idxs[1:][iou_vals < threshold]
    return keep


def init_trt_engine(path):
    if not os.path.exists(path):
        raise FileNotFoundError(f"Moteur TRT introuvable : {path}")
    TRT_LOGGER = trt.Logger(trt.Logger.WARNING)
    with open(path, 'rb') as f:
        runtime = trt.Runtime(TRT_LOGGER)
        engine  = runtime.deserialize_cuda_engine(f.read())
    if engine is None:
        raise RuntimeError(f"Désérialisation échouée : {path}")
    return engine, engine.create_execution_context()


def load_embeddings(path):
    if not os.path.exists(path):
        return {}
    try:
        data = np.load(path, allow_pickle=True)
        return {str(data["names"][i]): data["embeddings"][i] for i in range(len(data["names"]))}
    except Exception as e:
        print(f"Erreur chargement embeddings : {e}")
        return {}


def preprocess_arcface(img):
    if img.size == 0:
        return None
    img = cv2.resize(img, (112, 112), interpolation=cv2.INTER_AREA)
    img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
    img = (img.astype(np.float32) - 127.5) / 128.0
    return np.expand_dims(np.transpose(img, (2, 0, 1)), 0).astype(np.float32)


def extract_embedding_arcface(crop, af_ctx, af_input_name, af_output_name,
                               af_in_host, af_in_dev, af_out_host, af_out_dev, af_stream):
    """Extrait et retourne l'embedding normalisé (512,) d'un crop de visage, ou None."""
    inp = preprocess_arcface(crop)
    if inp is None:
        return None
    try:
        np.copyto(af_in_host, inp.flatten())
        cuda.memcpy_htod_async(af_in_dev, af_in_host, af_stream)
        af_ctx.set_tensor_address(af_input_name,  int(af_in_dev))
        af_ctx.set_tensor_address(af_output_name, int(af_out_dev))
        af_ctx.execute_async_v3(stream_handle=af_stream.handle)
        cuda.memcpy_dtoh_async(af_out_host, af_out_dev, af_stream)
        af_stream.synchronize()

        emb  = af_out_host.copy()
        norm = np.linalg.norm(emb)
        if norm < 1e-10:
            return None
        return emb / norm
    except Exception as e:
        print(f"Erreur extraction embedding : {e}")
        return None


# ============= NODE PRINCIPAL ===============

class QboFaceTrackingNode(Node):

    def __init__(self):
        super().__init__('qbo_face_tracking_node')
        self.get_logger().info("Initialisation QboFaceTrackingNode...")

        self.bridge = CvBridge()

        # --- Embeddings connus ---
        self.known_embeddings = load_embeddings(EMBEDDING_PATH)
        self.get_logger().info(f"Base embeddings : {len(self.known_embeddings)} identité(s)")

        # --- UltraFace TRT ---
        self.get_logger().info("Chargement UltraFace TRT...")
        self.uf_engine, self.uf_ctx = init_trt_engine(ENGINE_PATH)

        self._uf_input_name = next(
            n for n in self.uf_engine
            if self.uf_engine.get_tensor_mode(n) == trt.TensorIOMode.INPUT)
        self._uf_conf_name = next(
            n for n in self.uf_engine
            if self.uf_engine.get_tensor_mode(n) == trt.TensorIOMode.OUTPUT
            and self.uf_engine.get_tensor_shape(n)[-1] == 2)
        self._uf_loc_name = next(
            n for n in self.uf_engine
            if self.uf_engine.get_tensor_mode(n) == trt.TensorIOMode.OUTPUT
            and self.uf_engine.get_tensor_shape(n)[-1] == 4)

        uf_in_shape         = self.uf_engine.get_tensor_shape(self._uf_input_name)
        self._uf_conf_shape = self.uf_engine.get_tensor_shape(self._uf_conf_name)
        self._uf_loc_shape  = self.uf_engine.get_tensor_shape(self._uf_loc_name)

        self._uf_in_host   = cuda.pagelocked_empty(int(np.prod(uf_in_shape)),             np.float32)
        self._uf_in_dev    = cuda.mem_alloc(self._uf_in_host.nbytes)
        self._uf_conf_host = cuda.pagelocked_empty(int(np.prod(self._uf_conf_shape)),     np.float32)
        self._uf_conf_dev  = cuda.mem_alloc(self._uf_conf_host.nbytes)
        self._uf_loc_host  = cuda.pagelocked_empty(int(np.prod(self._uf_loc_shape)),      np.float32)
        self._uf_loc_dev   = cuda.mem_alloc(self._uf_loc_host.nbytes)
        self._uf_stream    = cuda.Stream()
        self._anchors      = generate_anchors(IMG_W, IMG_H)

        # --- ArcFace TRT ---
        self.get_logger().info("Chargement ArcFace TRT...")
        self.af_engine, self.af_ctx = init_trt_engine(ARCFACE_ENGINE_PATH)

        self._af_input_name = next(
            n for n in self.af_engine
            if self.af_engine.get_tensor_mode(n) == trt.TensorIOMode.INPUT)
        self._af_output_name = next(
            n for n in self.af_engine
            if self.af_engine.get_tensor_mode(n) == trt.TensorIOMode.OUTPUT)

        af_in_shape  = self.af_engine.get_tensor_shape(self._af_input_name)
        af_out_shape = self.af_engine.get_tensor_shape(self._af_output_name)

        self._af_in_host  = cuda.pagelocked_empty(int(np.prod(af_in_shape)),  np.float32)
        self._af_in_dev   = cuda.mem_alloc(self._af_in_host.nbytes)
        self._af_out_host = cuda.pagelocked_empty(int(np.prod(af_out_shape)), np.float32)
        self._af_out_dev  = cuda.mem_alloc(self._af_out_host.nbytes)
        self._af_stream   = cuda.Stream()

        # --- État interne ---
        self._reco_interval   = 1.0 / RECOGNITION_FPS
        self._last_reco_time  = 0.0
        self._last_det_time   = 0.0
        self._face_present    = False
        self._smoothed_center = None
        self._smoothed_size   = None   # (width, height) lissés comme le centre
        self._alpha           = 0.15
        self._current_name    = "Unknown"
        self._current_dist    = float('inf')
        self._current_crop    = None    # dernier crop détecté (utile pour capture externe)
        self._last_pub_name   = None
        self._last_nose_color = -1
        self._last_roi        = None    # dernier ROI calculé, publié à 30Hz par timer

        # --- Publishers / Subscribers ---
        self.roi_pub  = self.create_publisher(RegionOfInterest, "/target_roi",            10)
        self.rec_pub  = self.create_publisher(String,           "/is_recognized",         10)
        self.dbg_pub  = self.create_publisher(Image,            "/face_tracking_debug",   10)
        self.nose_pub = self.create_publisher(Nose,             "/qbo_arduqbo/nose_ctrl/cmd_nose", 10)

        self.create_subscription(Image, "/inverted_eye_image", self.image_callback, 10)

        self.get_logger().info("QboFaceTrackingNode prêt.")

    # ------------------------------------------------------------------ #
    #  DÉTECTION                                                           #
    # ------------------------------------------------------------------ #

    def _detect(self, frame):
        """Retourne {'bbox': (x1,y1,x2,y2), 'crop': np.array, 'score': float} ou None."""
        h, w    = frame.shape[:2]
        scale_x = w / IMG_W
        scale_y = h / IMG_H

        inp = preprocess_ultraface(frame).flatten()
        np.copyto(self._uf_in_host, inp)
        cuda.memcpy_htod_async(self._uf_in_dev, self._uf_in_host, self._uf_stream)
        self.uf_ctx.set_tensor_address(self._uf_input_name, int(self._uf_in_dev))
        self.uf_ctx.set_tensor_address(self._uf_conf_name,  int(self._uf_conf_dev))
        self.uf_ctx.set_tensor_address(self._uf_loc_name,   int(self._uf_loc_dev))
        self.uf_ctx.execute_async_v3(stream_handle=self._uf_stream.handle)
        cuda.memcpy_dtoh_async(self._uf_conf_host, self._uf_conf_dev, self._uf_stream)
        cuda.memcpy_dtoh_async(self._uf_loc_host,  self._uf_loc_dev,  self._uf_stream)
        self._uf_stream.synchronize()

        scores = self._uf_conf_host.reshape(self._uf_conf_shape)[0][:, 1]
        boxes  = decode_boxes(self._uf_loc_host.reshape(self._uf_loc_shape)[0], self._anchors)

        mask = scores > CONF_THRESH
        if not mask.any():
            return None

        boxes, scores = boxes[mask], scores[mask]
        keep = nms(boxes, scores, NMS_THRESH)
        if not keep:
            return None

        x1, y1, x2, y2 = boxes[keep[0]]
        x1 = max(0, min(w, int(x1 * scale_x)))
        y1 = max(0, min(h, int(y1 * scale_y)))
        x2 = max(0, min(w, int(x2 * scale_x)))
        y2 = max(0, min(h, int(y2 * scale_y)))

        if x2 <= x1 or y2 <= y1:
            return None

        crop = frame[y1:y2, x1:x2]
        if crop.size == 0 or crop.shape[0] < 30 or crop.shape[1] < 30:
            return None

        return {'bbox': (x1, y1, x2, y2), 'crop': crop, 'score': float(scores[keep[0]])}

    # ------------------------------------------------------------------ #
    #  RECONNAISSANCE                                                      #
    # ------------------------------------------------------------------ #

    def _recognize(self, crop):
        """Retourne (nom, distance). nom='Unknown' si distance >= RECO_THRESHOLD."""
        emb = extract_embedding_arcface(
            crop, self.af_ctx,
            self._af_input_name, self._af_output_name,
            self._af_in_host, self._af_in_dev,
            self._af_out_host, self._af_out_dev,
            self._af_stream
        )
        if emb is None:
            return ("Unknown", float('inf'))

        best_name, best_dist = "Unknown", float('inf')
        for name, db_emb in self.known_embeddings.items():
            if db_emb.shape != emb.shape:
                continue
            dist = 1.0 - float(np.clip(np.dot(emb, db_emb), -1.0, 1.0))
            if dist < best_dist:
                best_dist, best_name = dist, name

        if best_dist >= RECO_THRESHOLD:
            best_name = "Unknown"

        return (best_name, best_dist)

    # ------------------------------------------------------------------ #
    #  NEZ                                                                 #
    # ------------------------------------------------------------------ #

    def _set_nose(self, color: int):
        """Change la couleur du nez uniquement si différente (0=éteint, 1=rouge, 2=vert)."""
        if color != self._last_nose_color:
            msg = Nose()
            msg.color = color
            self.nose_pub.publish(msg)
            self._last_nose_color = color

    # ------------------------------------------------------------------ #
    #  CALLBACK PRINCIPAL                                                  #
    # ------------------------------------------------------------------ #

    def image_callback(self, msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f"Conversion image : {e}")
            return

        if frame.size == 0:
            return

        now  = time.monotonic()
        h, w = frame.shape[:2]

        # --- Détection (30fps) ---
        det = self._detect(frame)

        if det is None:
            # Pas de visage cette frame — attendre DETECTION_TIMEOUT avant de confirmer l'absence
            if self._face_present and (now - self._last_det_time > DETECTION_TIMEOUT):
                self._face_present    = False
                self._smoothed_center = None
                self._smoothed_size   = None
                self._current_crop    = None
                self._current_name    = "Unknown"
                self._current_dist    = float('inf')
                self._last_roi        = None
                self._publish_status("none")
                self._set_nose(0)  # Éteint

            # Publier ROI vide à chaque frame (width=0 = signal "perdu" pour le head tracker)
            empty_roi            = RegionOfInterest()
            empty_roi.width      = 0
            empty_roi.height     = 0
            empty_roi.do_rectify = False
            self.roi_pub.publish(empty_roi)

            if self.dbg_pub.get_subscription_count() > 0:
                ann = frame.copy()
                self._draw_overlay(ann)
                self.dbg_pub.publish(self.bridge.cv2_to_imgmsg(ann, encoding="bgr8"))
            return

        # --- Visage présent ---
        self._last_det_time  = now
        self._face_present   = True
        self._current_crop   = det['crop']   # exposé pour capture externe

        x1, y1, x2, y2 = det['bbox']
        cx = (x1 + x2) // 2
        cy = (y1 + y2) // 2

        # Lissage exponentiel du centre
        if self._smoothed_center is None:
            self._smoothed_center = (cx, cy)
        else:
            sx = int(self._alpha * cx + (1 - self._alpha) * self._smoothed_center[0])
            sy = int(self._alpha * cy + (1 - self._alpha) * self._smoothed_center[1])
            self._smoothed_center = (sx, sy)

        # Lissage exponentiel de la taille (évite les sauts bbox)
        rw, rh = x2 - x1, y2 - y1
        if self._smoothed_size is None:
            self._smoothed_size = (rw, rh)
        else:
            sw = int(self._alpha * rw + (1 - self._alpha) * self._smoothed_size[0])
            sh = int(self._alpha * rh + (1 - self._alpha) * self._smoothed_size[1])
            self._smoothed_size = (sw, sh)

        # --- Reconnaissance (2fps max) ---
        if now - self._last_reco_time >= self._reco_interval:
            self._current_name, self._current_dist = self._recognize(det['crop'])
            self._last_reco_time = now
            self.get_logger().debug(
                f"Reconnaissance : {self._current_name} (dist={self._current_dist:.3f})")

        # --- Publication ROI (centre + taille tous deux lissés) ---
        smw, smh = self._smoothed_size
        sx1 = max(0, self._smoothed_center[0] - smw // 2)
        sy1 = max(0, self._smoothed_center[1] - smh // 2)
        sx2 = min(w,  self._smoothed_center[0] + smw // 2)
        sy2 = min(h,  self._smoothed_center[1] + smh // 2)

        roi            = RegionOfInterest()
        roi.x_offset   = int(sx1)
        roi.y_offset   = int(sy1)
        roi.width      = int(sx2 - sx1)
        roi.height     = int(sy2 - sy1)
        roi.do_rectify = False
        self.roi_pub.publish(roi)

        # --- Statut reconnaissance + nez ---
        self._publish_status(self._current_name)
        if self._current_name != "Unknown":
            self._set_nose(2)  # Vert = reconnu
        else:
            self._set_nose(1)  # Rouge = inconnu

        # --- Image debug (seulement si un subscriber écoute) ---
        if self.dbg_pub.get_subscription_count() > 0:
            ann = frame.copy()

            # Rectangles
            cv2.rectangle(ann, (x1, y1), (x2, y2), (255, 0, 0), 1)                       # bbox brute (bleu, 1px)
            cv2.rectangle(ann, (int(sx1), int(sy1)), (int(sx2), int(sy2)), (0, 0, 0), 2)  # ROI lissé (noir, 2px)
            cv2.circle(ann, self._smoothed_center, 6, (0, 0, 255), -1)                    # centre (rouge)

            # Nom au-dessus du visage
            label = self._current_name
            if self._current_name != "Unknown":
                label += f" ({self._current_dist:.2f})"
            cv2.putText(ann, label, (x1, max(y1 - 10, 20)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 0), 2)

            # Titre + légende permanents
            self._draw_overlay(ann)

            self.dbg_pub.publish(self.bridge.cv2_to_imgmsg(ann, encoding="bgr8"))

    # ------------------------------------------------------------------ #
    #  OVERLAY DEBUG (titre + légende) — appelé sur chaque frame debug    #
    # ------------------------------------------------------------------ #

    def _draw_overlay(self, img):
        """Dessine le titre centré et la légende bas-droite sur l'image."""
        h, w = img.shape[:2]

        # Titre centré en haut
        title      = "FACE RECOGNITION"
        font       = cv2.FONT_HERSHEY_SIMPLEX
        font_scale = 0.8
        thickness  = 2
        (tw, _), _ = cv2.getTextSize(title, font, font_scale, thickness)
        cv2.putText(img, title, ((w - tw) // 2, 28), font, font_scale, (255, 255, 255), thickness)

        # Légende bas-droite
        leg_font  = cv2.FONT_HERSHEY_SIMPLEX
        leg_scale = 0.5
        leg_thick = 1
        pad    = 10   # marge bord image
        sq     = 12   # taille carré couleur
        gap    = 6    # espace carré→texte
        line_h = 22   # hauteur d'une ligne

        legends = [
            ((255, 0, 0), "Detection brute"),
            ((0,   0, 0), "ROI suivi (lisse)"),
        ]
        for i, (color, text) in enumerate(reversed(legends)):
            (lw, _), _ = cv2.getTextSize(text, leg_font, leg_scale, leg_thick)
            bx = w - pad - sq - gap - lw
            by = h - pad - i * line_h
            cv2.rectangle(img, (bx, by - sq), (bx + sq, by), color, -1)
            cv2.rectangle(img, (bx, by - sq), (bx + sq, by), (200, 200, 200), 1)  # bordure grise
            cv2.putText(img, text, (bx + sq + gap, by - 2),
                        leg_font, leg_scale, (0, 0, 0), leg_thick)  # texte noir

    # ------------------------------------------------------------------ #
    #  PUBLICATION STATUT                                                  #
    # ------------------------------------------------------------------ #

    def _publish_status(self, name: str):
        """Publie sur /is_recognized uniquement si le nom a changé."""
        if name != self._last_pub_name:
            msg      = String()
            msg.data = name
            self.rec_pub.publish(msg)
            self._last_pub_name = name
            self.get_logger().info(f"/is_recognized → {name}")

    # ------------------------------------------------------------------ #
    #  AJOUT D'UN EMBEDDING (appelable depuis un nœud / thread externe)   #
    # ------------------------------------------------------------------ #

    def add_embedding_to_database(self, name: str, embedding: np.ndarray):
        """
        Ajoute un embedding normalisé (512,) dans la base mémoire et sauvegarde le .npz.

        Exemple d'appel depuis un thread externe (voir le grand commentaire en haut du fichier
        pour le workflow complet de capture) :

            mean_emb = np.mean(np.array(embeddings_list), axis=0)
            norm     = np.linalg.norm(mean_emb)
            if norm > 1e-10:
                mean_emb /= norm
            node.add_embedding_to_database("NouvellePersonne", mean_emb)
        """
        try:
            self.known_embeddings[name] = embedding
            names          = list(self.known_embeddings.keys())
            embeddings_arr = np.array([self.known_embeddings[n] for n in names])
            np.savez_compressed(EMBEDDING_PATH, names=names, embeddings=embeddings_arr)
            self.get_logger().info(
                f"Embedding ajouté pour '{name}' — base : {len(names)} identité(s)")
        except Exception as e:
            self.get_logger().error(f"Erreur sauvegarde embedding : {e}")
            raise

    # ------------------------------------------------------------------ #
    #  NETTOYAGE GPU                                                       #
    # ------------------------------------------------------------------ #

    def destroy_node(self):
        for attr in ('_uf_in_dev', '_uf_conf_dev', '_uf_loc_dev',
                     '_af_in_dev', '_af_out_dev'):
            try:
                getattr(self, attr).free()
            except Exception:
                pass
        self.get_logger().info("Ressources GPU libérées.")
        super().destroy_node()


# ------------------------------------------------------------------ #

def main(args=None):
    rclpy.init(args=args)
    node = QboFaceTrackingNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()