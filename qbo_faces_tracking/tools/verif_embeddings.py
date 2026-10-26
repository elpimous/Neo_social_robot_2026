import numpy as np

def check_embeddings_file(path="/home/nvidia/Documents/face_embeddings.npz"):
    """Vérifie le contenu du fichier d'embeddings"""
    try:
        data = np.load(path, allow_pickle=True)
        print(f"[INFO] Fichier chargé : {path}")
        print(f"[INFO] Clés disponibles : {list(data.keys())}")
        
        if "embeddings" in data:
            embs = data["embeddings"]
            print(f"[INFO] Embeddings shape : {embs.shape}")
            print(f"[INFO] Embeddings dtype : {embs.dtype}")
            print(f"[INFO] Exemple embedding[0] (5 premiers) : {embs[0][:5]}")
            
        if "names" in data:
            names = data["names"]
            print(f"[INFO] Names shape : {names.shape}")
            print(f"[INFO] Names : {names}")
            
        # Vérifier si tous les embeddings sont identiques (problème courant)
        if "embeddings" in data and len(data["embeddings"]) > 1:
            embs = data["embeddings"]
            first = embs[0]
            all_same = all(np.allclose(first, embs[i]) for i in range(1, len(embs)))
            if all_same:
                print("[WARNING] ⚠️ Tous les embeddings sont identiques ! Problème probable.")
            else:
                print("[INFO] ✅ Les embeddings sont différents.")
                
    except Exception as e:
        print(f"[ERROR] Erreur lecture fichier : {e}")

# Lancer la vérification
check_embeddings_file()