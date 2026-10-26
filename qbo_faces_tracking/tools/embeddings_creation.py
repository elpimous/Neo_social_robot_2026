#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Génère une base d'embeddings moyens ArcFace-MobileFaceNet (TensorRT).
Chaque sous-dossier de datas/ correspond à une personne (ex: datas/Axel/*.jpg).
Optimisé et commenté pour Jetson Orin NX.
"""

import os
import cv2
import numpy as np
import tensorrt as trt
import pycuda.driver as cuda
import pycuda.autoinit

# ====== PARAMÈTRES ======
ARCFACE_ENGINE_PATH = "/home/nvidia/qbo_ws/src/qbo_faces_tracking/models/w600k_mbf_fp16.engine"
DATAS_ROOT = "/home/nvidia/qbo_ws/src/qbo_faces_tracking/datas/"
EMBEDDING_PATH = "/home/nvidia/qbo_ws/src/qbo_faces_tracking/face_embeddings/face_embeddings_average.npz"
FACE_SIZE = 112
EMBEDDING_SIZE = 512

# ====== INITIALISATION MOTEUR TRT ======
print("[INFO] Chargement ArcFace MobileFaceNet engine...")
TRT_LOGGER = trt.Logger(trt.Logger.WARNING)
with open(ARCFACE_ENGINE_PATH, 'rb') as f:
    engine_data = f.read()
arcface_runtime = trt.Runtime(TRT_LOGGER)
arcface_engine = arcface_runtime.deserialize_cuda_engine(engine_data)
arcface_context = arcface_engine.create_execution_context()

arcface_input_name = [arcface_engine.get_tensor_name(i)
    for i in range(arcface_engine.num_io_tensors)
    if arcface_engine.get_tensor_mode(arcface_engine.get_tensor_name(i)) == trt.TensorIOMode.INPUT][0]
arcface_output_name = [arcface_engine.get_tensor_name(i)
    for i in range(arcface_engine.num_io_tensors)
    if arcface_engine.get_tensor_mode(arcface_engine.get_tensor_name(i)) == trt.TensorIOMode.OUTPUT][0]

arcface_input_shape = arcface_engine.get_tensor_shape(arcface_input_name)
arcface_output_shape = arcface_engine.get_tensor_shape(arcface_output_name)

arcface_input_host = cuda.pagelocked_empty([int(np.prod(arcface_input_shape))], dtype=np.float32)
arcface_input_dev = cuda.mem_alloc(arcface_input_host.nbytes)
arcface_output_host = cuda.pagelocked_empty([int(np.prod(arcface_output_shape))], dtype=np.float32)
arcface_output_dev = cuda.mem_alloc(arcface_output_host.nbytes)
arcface_stream = cuda.Stream()

def preprocess_face_arcface(face_img):
    """
    Prépare une image visage pour ArcFace (112x112, RGB, normalisation, CHW).
    """
    img = cv2.resize(face_img, (FACE_SIZE, FACE_SIZE))
    img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
    img = (img.astype(np.float32) - 127.5) / 128.0
    img = np.transpose(img, (2, 0, 1))  # HWC -> CHW
    return np.expand_dims(img, axis=0).astype(np.float32)  # (1,3,112,112)

def is_image_file(filename):
    """
    Vérifie si le fichier est une image supportée.
    """
    ext = filename.lower().split('.')[-1]
    return ext in ['jpg', 'jpeg', 'png', 'bmp']

def extract_embedding_arcface(face_img):
    """
    Extrait un vecteur embedding (512D) normalisé d’un visage.
    """
    face_preproc = preprocess_face_arcface(face_img).flatten()
    np.copyto(arcface_input_host, face_preproc)
    cuda.memcpy_htod_async(arcface_input_dev, arcface_input_host, arcface_stream)
    arcface_context.set_tensor_address(arcface_input_name, int(arcface_input_dev))
    arcface_context.set_tensor_address(arcface_output_name, int(arcface_output_dev))
    arcface_context.execute_async_v3(stream_handle=arcface_stream.handle)
    cuda.memcpy_dtoh_async(arcface_output_host, arcface_output_dev, arcface_stream)
    arcface_stream.synchronize()
    emb = arcface_output_host.reshape(arcface_output_shape)
    emb_norm = emb / (np.linalg.norm(emb) + 1e-10)
    return emb_norm.flatten()

def create_mean_embeddings_database(root_dir, output_path):
    """
    Pour chaque personne (sous-dossier), extrait l’embedding de chaque image valide,
    fait la moyenne puis normalise, et stocke un seul embedding par personne.
    """
    mean_embeddings = []
    names = []
    n_total = 0
    n_errors = 0

    for person_name in sorted(os.listdir(root_dir)):
        person_dir = os.path.join(root_dir, person_name)
        if not os.path.isdir(person_dir):
            continue
        print(f"\n[INFO] --- Traitement : {person_name} ---")
        embs = []
        for img_file in sorted(os.listdir(person_dir)):
            if not is_image_file(img_file):
                continue
            img_path = os.path.join(person_dir, img_file)
            img = cv2.imread(img_path)
            emb = extract_embedding_arcface(img)
            if np.any(np.isnan(emb)) or np.any(np.isinf(emb)):
                print(f"[ERREUR] Embedding NaN/Inf pour {img_path}")
                n_errors += 1
                continue
            embs.append(emb)
            n_total += 1
            print(f"[OK] {img_file}")

        # Calcule embedding moyen normalisé pour la personne
        if len(embs) > 0:
            mean_emb = np.mean(embs, axis=0)
            mean_emb = mean_emb / (np.linalg.norm(mean_emb) + 1e-10)
            mean_embeddings.append(mean_emb)
            names.append(person_name)
            print(f"[INFO] Embedding moyen OK pour {person_name}")
        else:
            print(f"[WARN] Aucun embedding valide pour {person_name}")

    # Sauvegarde la base (embedding moyen par personne)
    mean_embeddings = np.array(mean_embeddings, dtype=np.float32)
    names = np.array(names)
    np.savez(output_path, embeddings=mean_embeddings, names=names)
    print(f"\n[INFO] Sauvegardé : {output_path}")
    print(f"[INFO] Nombre de personnes : {len(mean_embeddings)}")
    print(f"[INFO] Nombre d'images traitées : {n_total}")
    print(f"[INFO] Erreurs de lecture/embedding : {n_errors}")

if __name__ == "__main__":
    create_mean_embeddings_database(DATAS_ROOT, EMBEDDING_PATH)
