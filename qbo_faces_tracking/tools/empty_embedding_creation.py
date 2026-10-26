import numpy as np

# Chemin de sortie du fichier .npz vide
output_path = "/home/nvidia/qbo_ws/src/qbo_faces_tracking/face_embeddings/face_embeddings.npz"

# Crée des tableaux vides avec les bonnes formes
embeddings = np.empty((0, 512), dtype=np.float32)  # 0 vecteurs d'embedding de 512 dimensions
names = np.empty((0,), dtype='<U1')  # 0 noms, type string (unicode d'au moins 1 caractère)

# Sauvegarde dans un .npz
np.savez(output_path, embeddings=embeddings, names=names)

print(f"[INFO] Base d'embeddings vierge créée à : {output_path}")
