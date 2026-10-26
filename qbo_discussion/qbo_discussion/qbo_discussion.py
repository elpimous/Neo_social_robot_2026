#!/usr/bin/env python3
"""
Qbo Discussion Node
Optimisé et commenté pour ROS2 Humble sur Orin NX
"""

import sys
import os
import json
import time
import random

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

import numpy as np
import faiss
from sentence_transformers import SentenceTransformer
from sklearn.metrics.pairwise import cosine_similarity

# Détection et configuration du périphérique GPU/CPU
import torch


def get_compute_device(min_cuda_capability=(9, 0)):
    """
    Retourne le meilleur device disponible ('cuda' ou 'cpu').
    Vérifie la compatibilité CUDA capability pour Orin NX (sm_87 non supporté).
    """
    if torch.cuda.is_available():
        major, minor = torch.cuda.get_device_capability(0)
        if (major, minor) >= min_cuda_capability:
            return 'cuda'
        else:
            print(f"⚠️ Cuda device sm_{major}{minor} non supporté, bascule sur CPU")
    return 'cpu'


device = get_compute_device()


class NeoRAG:
    """
    Classe RAG (Retrieval-Augmented Generation) optimisée :
    - Chargement modèle en FP16 sur GPU si possible
    - Index FAISS Inner Product pour similarité cosinus
    """
    def __init__(self, json_path: str, model_name: str = "intfloat/multilingual-e5-large"):
        print("🤖 Initialisation de Neo RAG...")
        start_time = time.time()

        # Chargement du modèle SentenceTransformer
        self.model = SentenceTransformer(model_name, device=device)
        if device == 'cuda':
            # Passage en half precision pour économiser la VRAM
            self.model.half()

        # Chargement des données Q&A depuis fichier JSON
        with open(json_path, 'r', encoding='utf-8') as f:
            self.data = json.load(f)

        # Création des embeddings pour toutes les questions
        questions = [item['question'] for item in self.data]
        self.embeddings = self.model.encode(
            questions,
            convert_to_tensor=True,
            normalize_embeddings=True,
            show_progress_bar=False
        )

        # Construction de l'index FAISS (Inner Product) adapté embeddings normalisés
        dim = self.embeddings.shape[1]
        # FAISS exige float32
        cpu_embeddings = self.embeddings.cpu().numpy().astype(np.float32)
        self.index = faiss.IndexFlatIP(dim)
        self.index.add(cpu_embeddings)

        elapsed = time.time() - start_time
        print(f"✅ Prêt en {elapsed:.2f}s - {len(self.data)} entrées indexées")

    def query(self, question: str, top_k: int = 1, confidence_threshold: float = 0.80) -> dict:
        """
        Recherche la réponse la plus pertinente en comparant embeddings.

        :param question: phrase utilisateur
        :param top_k: nombre de réponses retournées
        :param confidence_threshold: seuil minimal de similarité
        :return: dict avec résultats et indicateurs
        """
        start_time = time.time()

        # Embedding de la question
        q_embed = self.model.encode(
            [question],
            convert_to_tensor=True,
            normalize_embeddings=True
        )

        # Recherche dans l'index FAISS (Inner Product ~ similarité cosinus)
        cpu_q = q_embed.cpu().numpy().astype(np.float32)
        scores, indices = self.index.search(cpu_q, top_k)

        # Filtrage des résultats selon le seuil de confiance
        results = []
        for rank, (idx, score) in enumerate(zip(indices[0], scores[0]), start=1):
            if score >= confidence_threshold:
                entry = self.data[idx]
                results.append({
                    'rank': rank,
                    'question': entry['question'],
                    'answer': entry['answer'],
                    'similarity': float(score)
                })

        return {
            'results': results,
            'confidence_met': len(results) > 0,
            'max_score': float(scores[0][0]) if scores.size > 0 else 0.0,
            'latency': time.time() - start_time
        }


class QboDiscussionNode(Node):
    """
    Node ROS2 pour gérer les interactions Q&A via RAG.
    Écoute /listened, publie sur /tts_input.
    """
    def __init__(self):
        super().__init__('qbo_discussion_node')

        # Déclaration des paramètres ROS configurables via YAML
        self.declare_parameter('json_path', '/home/nvidia/qbo_ws/src/qbo_discussion/qbo_discussion/rag/neo_json_random.json')
        self.declare_parameter('confidence_threshold', 0.80)
        self.declare_parameter('model_name', 'intfloat/multilingual-e5-large')
        self.declare_parameter('user_name', 'Vincent')

        # Récupération des paramètres
        json_path = self.get_parameter('json_path').value
        self.confidence_threshold = self.get_parameter('confidence_threshold').value
        model_name = self.get_parameter('model_name').value
        self.user_name = self.get_parameter('user_name').value

        # Initialisation de NeoRAG avec gestion d'erreurs
        try:
            self.rag = NeoRAG(json_path, model_name)
            self.get_logger().info(f"🤖 RAG initialisé (seuil={self.confidence_threshold})")
        except Exception as err:
            self.get_logger().error(f"❌ Échec init RAG: {err}")
            self.rag = None

        # Subscriber et Publisher
        self.subscription = self.create_subscription(
            String,
            '/listened',
            self.listener_callback,
            10
        )
        self.publisher = self.create_publisher(String, '/tts_input', 10)

        self.get_logger().info("🔍 Node démarré, écoute /listened, publie /tts_input")

    def process_answer(self, raw: str) -> str:
        """
        Traite la réponse brute : choix aléatoire, remplacements.
        """
        # Choix aléatoire si plusieurs options séparées par '|'
        text = random.choice(raw.split('|')).strip() if '|' in raw else raw.strip()
        # Remplacements de variables
        text = text.replace('<USER_ID>', self.user_name)
        # TODO: remplacer dynamiquement TIME_ID et METEO_ID
        text = text.replace('<TIME_ID>', '<TIME_ID>')
        text = text.replace('<METEO_ID>', '<METEO_ID>')
        return text

    def listener_callback(self, msg: String):
        question = msg.data.strip()

        # --- Garde 1 : message vide ou ponctuation seule ---
        if not question or all(c in '.,;:!?…- ' for c in question):
            self.get_logger().warn(f"⚠️ Input ignoré (ponctuation/vide): '{question}'")
            return

        # --- Garde 2 : trop court pour être une vraie question ---
        words = question.split()
        if len(words) < 2 or len(question) < 5:
            self.get_logger().warn(f"⚠️ Input trop court ignoré: '{question}'")
            return
    
        self.get_logger().info(f"❓ Question: {question}")
        if self.rag is None:
            self.get_logger().error("❌ RAG non initialisé")
            return

        try:
            response = self.rag.query(
                question,
                top_k=1,
                confidence_threshold=self.confidence_threshold
            )
            if response['confidence_met']:
                best = response['results'][0]
                self.get_logger().info(f"📋 Similarité: {best['similarity']:.3f}")
                answer = self.process_answer(best['answer'])
            else:
                self.get_logger().warn(
                    f"❌ Aucun résultat ≥ {self.confidence_threshold} (max={response['max_score']:.3f})"
                )
                answer = "Je n'ai pas bien compris. Peux-tu reformuler ?"

            # Publication
            out = String()
            out.data = answer
            self.publisher.publish(out)
            self.get_logger().info(f"✅ Réponse envoyée: {answer}")
            time.sleep(1)

        except Exception as e:
            self.get_logger().error(f"❌ Erreur traitement: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = QboDiscussionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
