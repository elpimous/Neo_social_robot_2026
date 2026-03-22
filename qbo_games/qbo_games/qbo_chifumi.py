#!/usr/bin/env python3
# -*- coding: utf-8 -*-

# =======================================================================================================
#     Neo CHIFUMI PROGRAM - USING MINASI ALGORITHM + MEDIAPIPE HAND DETECTION (ROS2 VERSION)
#     
#     DESCRIPTION:
#     Ce programme implémente un jeu de pierre-papier-ciseaux intelligent pour le robot Neo.
#     Il combine deux technologies principales:
#     1. L'algorithme de Minasi pour prédire les coups de l'adversaire
#     2. MediaPipe pour détecter les gestes de la main en temps réel
#     
#     FONCTIONNALITÉS:
#     - Détection automatique des gestes pierre/papier/ciseaux via la caméra
#     - Prédiction intelligente des coups futurs basée sur l'historique
#     - Synthèse vocale pour l'interaction naturelle
#     - Système de points (10 points chacun au départ)
#     - Gestion robuste des erreurs et timeouts
#
#     OPTIMISATIONS JETSON ORIN NX:
#     - MediaPipe model_complexity=1 pour de meilleures reco. (mais plus lent !)
#     - Traitement d'image optimisé
#     - Gestion mémoire efficace
#
#     RÉFÉRENCE ALGORITHME:
#     https://www.areaprog.com/algo/article-380-algorithme-de-minasi-utilisation-de \
#     -l-algorithme-de-minasi-pour-prevoir-le-choix-d-un-joueur-de-self.pierre-self.papier-self.ciseaux
#
#     AUTEURS:
#     elpimous12 - Août 2018 (version originale)
#     Conversion ROS2 + MediaPipe - Août 2025
# =======================================================================================================


# =======================================================================================================
# IMPORTS - Toutes les bibliothèques nécessaires
# =======================================================================================================

import random      # Pour les choix aléatoires du robot et la variété des phrases
import numpy as np # Pour les calculs numériques (utilisé par MediaPipe)
from time import sleep, time  # sleep: pauses dans le jeu, time: mesure temporelle

# Imports ROS2 - Framework robotique
import rclpy                    # Cœur de ROS2 Python
from rclpy.node import Node     # Classe de base pour créer un nœud ROS2
from sensor_msgs.msg import Image    # Type de message pour les images de caméra
from std_msgs.msg import String      # Type de message pour les chaînes de caractères
from cv_bridge import CvBridge       # Convertit entre formats ROS2 et OpenCV

# Imports MediaPipe - Intelligence artificielle pour détection de main
import mediapipe as mp # Bibliothèque Google pour reconnaissance de gestes
import cv2             # OpenCV pour traitement d'images

# Imports pour la synthèse vocale ROS2 (via topic publisher)
# Plus besoin d'imports pour les services


# =======================================================================================================
# CLASSE PRINCIPALE - Nœud ROS2 qui gère tout le jeu
# =======================================================================================================

class GameNode(Node):
    """
    Classe principale qui hérite de Node (ROS2).
    Cette classe gère:
    - La détection de gestes MediaPipe
    - L'algorithme de prédiction Minasi
    - La logique de jeu pierre-papier-ciseaux
    - L'interaction vocale avec l'utilisateur
    - La communication ROS2
    """

    def __init__(self):
        """
        Constructeur de la classe GameNode.
        Initialise tous les composants nécessaires au jeu.
        """
        # Initialisation du nœud ROS2 avec un nom unique
        super().__init__('neo_chifumi_mediapipe_node')
        
        # =======================================================================================================
        # VARIABLES DE JEU - État et logique du pierre-papier-ciseaux
        # =======================================================================================================
        
        # Genre pour personnaliser les interactions (sera mis à jour par reconnaissance faciale)
        self.genre = "Vincent"

        # Chaîne qui stocke l'historique des coups pour l'algorithme Minasi
        # Format: "RpPsRr..." où majuscules=robot, minuscules=joueur
        self.pattern = ""
        
        # Définition des combinaisons gagnantes pour le robot
        # "Rp" = Robot Pierre vs Joueur papier = Robot perd
        self.coups_gagnants = ["Rp","Ps","Sr"] # Rock vs Paper, Paper vs Scissors, Scissors vs Rock
        
        # Noms des trois gestes possibles (utilisés pour la synthèse vocale)
        self.nom_des_formes = ['pierre', 'papier', 'ciseaux']
        
        # Combinaisons d'égalité (même geste des deux côtés)
        self.coups_nuls = ["Rr","Pp","Ss"]  # Rock-Rock, Paper-Paper, Scissors-Scissors
        
        # Variables pour stocker le dernier coup joué
        self.dernier_coup = ""    # Dernière combinaison robot+joueur (ex: "Rp")
        self.coup_letter = ""     # Lettre du coup joueur actuel ('r', 'p', 's' ou "")
        self.coup = ""           # Nom du coup joueur actuel ("pierre", "papier", "ciseaux" ou "")

        # Compteurs pour varier les phrases selon les séquences de victoires/défaites
        self.loose = 0  # Nombre de défaites consécutives du robot
        self.win = 0    # Nombre de victoires consécutives du robot  
        self.nul = 0    # Nombre d'égalités consécutives
        self.bad_step = 0  # Nombre d'échecs de détection consécutifs

        # =======================================================================================================
        # CONFIGURATION MEDIAPIPE - Détection de main optimisée pour Jetson
        # =======================================================================================================
        
        # Initialisation des modules MediaPipe
        self.mp_hands = mp.solutions.hands      # Module de détection de mains
        
        # Configuration du détecteur de mains avec optimisations Jetson
        self.hands = self.mp_hands.Hands(
            static_image_mode=False,      # Mode vidéo (plus rapide que image statique)
            max_num_hands=1,              # Une seule main maximum (économise les ressources)
            min_detection_confidence=0.7, # Seuil de confiance pour détecter une main
            min_tracking_confidence=0.5,  # Seuil de confiance pour suivre une main
            model_complexity=1            # Modèle le plus simple (0) pour vitesse au detriment de la perf.
            )
        
        # Utilitaire pour dessiner les landmarks (points de la main)
        self.mp_drawing = mp.solutions.drawing_utils
        
        # =======================================================================================================
        # VARIABLES DE GESTION DES GESTES - Stabilité et validation
        # =======================================================================================================
        
        # Geste actuellement détecté
        self.current_gesture = "AUCUNE_MAIN"
        
        # Niveau de confiance du geste actuel (0.0 à 1.0)
        self.gesture_confidence = 0.0
        
        # Timestamp du début de détection d'un geste stable
        self.gesture_stable_time = 0.0
        
        # Timestamp de la dernière validation de geste
        self.last_gesture_time = 0.0
        
        # Durée minimale (en secondes) qu'un geste doit être stable pour être validé
        # Évite les fausses détections lors des transitions de gestes
        self.gesture_lock_duration = 0.3 #1.0
        
        # =======================================================================================================
        # CONFIGURATION ROS2 - Communication et services
        # =======================================================================================================
        
        # Pont pour convertir entre les formats d'image ROS2 et OpenCV
        self.cv_bridge = CvBridge()

        # Abonnement au flux vidéo de la caméra du robot
        # Topic '/inverted_eye_image' = image de l'œil du robot (inversée)
        self.image_subscription = self.create_subscription(
            Image,                    # Type de message
            '/inverted_eye_image',    # Nom du topic
            self.image_callback,      # Fonction appelée à chaque nouvelle image
            10)                       # Taille de la queue (garde 10 images max)

        # Publisher optionnel pour debug visuel
        # Publie une image avec les landmarks dessinés et infos de détection
        self.debug_publisher = self.create_publisher(
            Image,              # Type de message
            '/chifoumi_debug',  # Nom du topic
            10)                 # Taille de la queue

        # Client pour le service de synthèse vocale
        self.speech_publisher = self.create_publisher(
            String,           # Type de message pour le texte à prononcer
            '/to_speak',      # Topic de synthèse vocale
            10)  


        # =======================================================================================================
        # PHRASES PRÉDÉFINIES - Variété dans les interactions vocales
        # =======================================================================================================
        
        # Phrases quand le joueur gagne (robot perd)
        self.win_sentences = [', Bravo, tu gagnes,',', Bien joué',', gagné',', bravo',
                             ', tu as gagné, bravo!',", j'ai perdu",', je viens de perdre']
        
        # Phrases quand le joueur perd (robot gagne)  
        self.loose_sentences = [', Tu as perdu',', tu viens de perdre',', Perdu',
                               ', oh; tu as perdu',", J'ai gagné",", C'est moi qui gagne",", je gagne"]
        
        # Phrases pour victoires répétées du joueur
        self.win_again_sentences = [", j'ai encore perdu",", tu as encore gagné",
                                   ", tu viens encore de gagner"]
        
        # Phrases pour victoires répétées du robot
        self.loose_again_sentences = [", j'ai encore gagné",", tu as encore perdu",
                                     ", tu viens encore de perdre"]
        
        # Phrases pour les égalités
        self.nul_sentences = [", pareil, on recommence",", égalité, on recommence",
                             ", égalité, recommençons",", pareil, recommençons"]
        
        # Phrases pour égalités répétées
        self.nul_again_sentences = ["encore un nul. Recommençons", "Encore une égalité, rejouons",
                                   "égalité, encore. on rejoue"]
        
        # Phrases de fin de partie
        self.stop_sentences = ["Merci d'avoir joué avec moi, "+self.genre+". C'était très sympas.",
                              "Je ferme le jeu et te remercie d'avoir joué avec moi "+self.genre]

        # Log d'initialisation terminée
        self.get_logger().info('Neo Chifumi MediaPipe node initialized - 02 Août 2025')

    # =======================================================================================================
    # CLASSIFICATION DES GESTES - Algorithme de reconnaissance pierre/papier/ciseaux
    # =======================================================================================================

    def classify_gesture(self, landmarks):
        """
        Analyse les landmarks MediaPipe pour classifier le geste de la main.
        
        Args:
            landmarks: Objet MediaPipe contenant les 21 points de la main
            
        Returns:
            tuple: (nom_du_geste, niveau_de_confiance)
                   nom_du_geste: "PIERRE", "PAPIER", "CISEAUX", ou "INCONNU"
                   niveau_de_confiance: float entre 0.0 et 1.0
        
        Principe:
            - MediaPipe détecte 21 landmarks sur la main (bout des doigts, articulations, etc.)
            - On compare les positions pour déterminer si chaque doigt est levé ou baissé
            - Classification basée sur le nombre et la combinaison de doigts levés
        """
        
        # Conversion des landmarks MediaPipe en liste de coordonnées [x, y]
        landmark_list = []
        for lm in landmarks.landmark:
            landmark_list.append([lm.x, lm.y])  # x,y normalisés entre 0 et 1
        
        # Liste pour stocker l'état de chaque doigt (1=levé, 0=baissé)
        fingers_up = []
        
        # =======================================================================================================
        # DÉTECTION DU POUCE - Cas spécial (horizontal vs vertical)
        # =======================================================================================================
        
        # Le pouce se déplace horizontalement, pas verticalement comme les autres doigts
        # Landmark 4 = bout du pouce, Landmark 3 = articulation du pouce
        if landmark_list[4][0] > landmark_list[3][0]:  # Main droite
            fingers_up.append(1)  # Pouce levé
        else:
            fingers_up.append(0)  # Pouce baissé
            
        # =======================================================================================================
        # DÉTECTION DES AUTRES DOIGTS - Comparaison verticale
        # =======================================================================================================
        
        # Landmarks des bouts de doigts (index, majeur, annulaire, auriculaire)
        finger_tips = [8, 12, 16, 20]
        # Landmarks des articulations correspondantes  
        finger_pips = [6, 10, 14, 18]
        
        # Pour chaque doigt, comparer la position Y du bout vs articulation
        for tip, pip in zip(finger_tips, finger_pips):
            if landmark_list[tip][1] < landmark_list[pip][1]:  # Y plus petit = plus haut
                fingers_up.append(1)  # Doigt levé
            else:
                fingers_up.append(0)  # Doigt baissé
        
        # Nombre total de doigts levés
        total_fingers = sum(fingers_up)
        
        # =======================================================================================================
        # LOGIQUE DE CLASSIFICATION - Règles de décision
        # =======================================================================================================
        
        # PIERRE: 0 ou 1 doigt levé (poing fermé ou presque fermé)
        if total_fingers ==0:
            return "PIERRE", 1.0
        if total_fingers == 1:
            return "PIERRE", 0.75
            
        # PAPIER: 4 ou 5 doigts levés (main ouverte)
        elif total_fingers == 5:
            return "PAPIER", 1.0
        elif total_fingers == 4:
            return "PAPIER", 0.75
            
        # CISEAUX: 2-3 doigts levés ET index ET majeur levés
        elif total_fingers == 2 and fingers_up[1] == 1 and fingers_up[2] == 1:
            return "CISEAUX", 1.0   
        elif total_fingers < 4 and fingers_up[1] == 1 and fingers_up[2] == 1:
            return "CISEAUX", 0.75
        
        # Geste non reconnu ou ambigu
        return "INCONNU", 0.3

    # =======================================================================================================
    # CALLBACK IMAGE - Traitement en temps réel du flux vidéo
    # =======================================================================================================

    def image_callback(self, msg):
        """Traite l'image et détecte les gestes de la main - ADAPTÉ AU SUIVI DE VISAGE"""
        try:
            current_time = time()
            
            # Conversion ROS2 vers OpenCV
            cv_image = self.cv_bridge.imgmsg_to_cv2(msg, "bgr8")
            
            # Conversion BGR vers RGB pour MediaPipe
            rgb_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)
            
            # Traitement avec MediaPipe
            results = self.hands.process(rgb_image)
            
            gesture_detected = "AUCUNE_MAIN"
            confidence = 0.0
            
            # Analyse des landmarks
            if results.multi_hand_landmarks:
                for hand_landmarks in results.multi_hand_landmarks:
                    # Classification du geste
                    gesture_detected, confidence = self.classify_gesture(hand_landmarks)
                    break  # Une seule main maximum
            
            # NOUVELLE LOGIQUE ADAPTÉE AU SUIVI DE VISAGE
            # Gestion de la stabilité du geste (assouplie pour le mouvement de caméra)
            if gesture_detected != "AUCUNE_MAIN" and gesture_detected == self.current_gesture and confidence > 0.8:
                # Même geste valide détecté, continuer le timer
                if self.gesture_stable_time == 0.0:
                    # Première détection de ce geste stable, démarrer le timer
                    self.gesture_stable_time = current_time
            elif gesture_detected != "AUCUNE_MAIN" and confidence > 0.8:
                # Nouveau geste valide détecté, démarrer immédiatement
                self.current_gesture = gesture_detected
                self.gesture_confidence = confidence
                self.gesture_stable_time = current_time  # Démarrer le timer immédiatement
            else:
                # Pas de main ou confiance trop faible
                if gesture_detected == "AUCUNE_MAIN":
                    # Reset seulement si vraiment aucune main
                    self.current_gesture = gesture_detected
                    self.gesture_confidence = confidence
                    self.gesture_stable_time = 0.0
                # Si confiance faible mais main détectée, on garde l'état actuel
            
            # Si le geste est stable depuis assez longtemps, on l'utilise
            if (self.gesture_stable_time > 0.0 and 
                current_time - self.gesture_stable_time >= self.gesture_lock_duration):
                
                # Conversion vers les lettres utilisées par l'algorithme Minasi
                if self.current_gesture == "PIERRE":
                    self.coup_letter = "r"
                    self.coup = "pierre"
                elif self.current_gesture == "PAPIER":
                    self.coup_letter = "p"
                    self.coup = "papier"
                elif self.current_gesture == "CISEAUX":
                    self.coup_letter = "s"
                    self.coup = "ciseaux"
                else:
                    self.coup_letter = ""
                    self.coup = ""
                
                # Reset pour éviter les détections multiples
                self.gesture_stable_time = 0.0
                self.last_gesture_time = current_time
            
            # Debug visuel (optionnel)
            if self.debug_publisher.get_subscription_count() > 0:
                debug_image = cv_image.copy()
                
                # Dessine les landmarks si détectés
                if results.multi_hand_landmarks:
                    for hand_landmarks in results.multi_hand_landmarks:
                        self.mp_drawing.draw_landmarks(
                            debug_image, hand_landmarks, self.mp_hands.HAND_CONNECTIONS)
                
                # Affichage des informations
                font = cv2.FONT_HERSHEY_SIMPLEX
                color = (0, 255, 0) if confidence > 0.8 else (0, 0, 255)
                
                cv2.putText(debug_image, f"Geste: {gesture_detected}", 
                           (10, 30), font, 0.7, color, 2)
                cv2.putText(debug_image, f"Confiance: {confidence:.1%}", 
                           (10, 60), font, 0.7, color, 2)
                cv2.putText(debug_image, f"Coup actuel: {self.coup}", 
                           (10, 90), font, 0.7, (255, 255, 0), 2)
                
                # Affichage du timer de stabilité
                if self.gesture_stable_time > 0.0:
                    stable_duration = current_time - self.gesture_stable_time
                    cv2.putText(debug_image, f"Stabilité: {stable_duration:.1f}s", 
                               (10, 120), font, 0.7, (255, 0, 255), 2)
                
                # Publication de l'image debug
                debug_msg = self.cv_bridge.cv2_to_imgmsg(debug_image, "bgr8")
                debug_msg.header = msg.header
                self.debug_publisher.publish(debug_msg)
                
        except Exception as e:
            self.get_logger().error(f'Erreur dans image_callback: {e}')

    # =======================================================================================================
    # SYNTHÈSE VOCALE - Communication avec l'utilisateur
    # =======================================================================================================

    def speak_this(self, text):
        """
        Fait parler le robot en publiant sur le topic /to_speak.
        
        Args:
            text (str): Texte à prononcer en français
        """
        # Créer le message String avec le texte à prononcer
        msg = String()
        msg.data = text
        
        # Publier le message sur le topic de synthèse vocale
        self.speech_publisher.publish(msg)
        
        # Log pour debug/monitoring
        self.get_logger().info(f'Speaking: {text}')

    # =======================================================================================================
    # ALGORITHME DE MINASI - Prédiction intelligente des coups
    # =======================================================================================================

    def getNextHit(self):
        """
        Implémente l'algorithme de Minasi pour prédire le prochain coup du joueur.
        
        Principe de l'algorithme:
            1. Analyser l'historique des coups (self.pattern)
            2. Chercher des séquences répétitives dans le comportement du joueur
            3. Prédire le prochain coup basé sur les patterns trouvés
            4. Choisir la réponse optimale pour battre cette prédiction
            
        Returns:
            str: "R" (Pierre), "P" (Papier), ou "S" (Ciseaux) - choix du robot
            
        Détail de l'algorithme:
            - Pour chaque longueur de séquence (de la plus longue à la plus courte)
            - Chercher si cette séquence finale apparaît ailleurs dans l'historique
            - Si oui, regarder ce qui a suivi dans le passé
            - Compter les occurrences de chaque coup suivant
            - Prédire le coup le plus probable et choisir la contre-attaque
        """
        
        # Initialisation des compteurs de prédiction
        self.pierre = 0   # Nombre de fois où "pierre" a suivi le pattern
        self.papier = 0   # Nombre de fois où "papier" a suivi le pattern  
        self.ciseaux = 0  # Nombre de fois où "ciseaux" a suivi le pattern

        # =======================================================================================================
        # CŒUR DE L'ALGORITHME MINASI - Recherche de patterns
        # =======================================================================================================
        
        # Parcourir toutes les longueurs de séquences possibles (de la plus longue à la plus courte)
        for i in reversed(range(len(self.pattern))): 
            # Séquence à chercher = fin de l'historique depuis la position i
            self.sequenceAChercher = self.pattern[i:]
            
            # Chaîne à analyser = début de l'historique jusqu'à la position i
            self.chaineAnalysee = self.pattern[:i]
            
            # Liste pour stocker les coups qui ont suivi chaque occurrence du pattern
            self.lstOccurence = []
            
            # Chercher toutes les occurrences de la séquence dans l'historique
            for x in range(0, (len(self.pattern)-len(self.sequenceAChercher))):
                # Si on trouve la séquence à la position x
                if self.pattern[x:(x+len(self.sequenceAChercher))] == self.sequenceAChercher:
                    # Récupérer les 2 coups qui ont suivi (robot + joueur)
                    suite = self.pattern[x+len(self.sequenceAChercher):(x+len(self.sequenceAChercher)+2)]
                    self.lstOccurence.append(suite)
            
            # Si aucune occurrence trouvée pour cette longueur, essayer une séquence plus courte
            self.occurence = self.lstOccurence
            if len(self.occurence) == 0:
                break
            else:
                # Reset des compteurs pour cette longueur de séquence
                self.papier = 0
                self.pierre = 0
                self.ciseaux = 0

            # =======================================================================================================
            # COMPTAGE DES OCCURRENCES - Analyse statistique
            # =======================================================================================================
            
            # Pour chaque occurrence trouvée, compter le coup du joueur qui a suivi
            for x in range(0, len(self.occurence)):
                # Le coup du joueur est en position [1] (position [0] = coup du robot)
                if self.occurence[x][1] == 'p':    # papier
                    self.papier += 1
                if self.occurence[x][1] == 'r':    # pierre (rock)
                    self.pierre += 1
                if self.occurence[x][1] == 's':    # ciseaux (scissors)
                    self.ciseaux += 1

        # =======================================================================================================
        # DÉCISION FINALE - Choix du coup du robot
        # =======================================================================================================
        
        # Si aucune prédiction possible (début de partie ou pattern unique)
        if self.pierre == 0 and self.papier == 0 and self.ciseaux == 0:
            return random.choice(["P", "R", "S"])  # Choix aléatoire

        # Trouver la prédiction avec le score maximum
        maxi = max(self.pierre, max(self.papier, self.ciseaux))
        
        # Liste des coups possibles du robot (contre-attaques)
        self.possibilite = ["P", "R", "S"]  # P bat r, R bat s, S bat p
        
        # Éliminer les choix non optimaux
        if (self.pierre != maxi):      # Si pierre n'est pas le plus probable
            del self.possibilite[self.possibilite.index("P")]  # Enlever Papier (qui bat pierre)
        if (self.ciseaux != maxi):     # Si ciseaux n'est pas le plus probable  
            del self.possibilite[self.possibilite.index("R")]  # Enlever Pierre (qui bat ciseaux)
        if (self.papier != maxi):      # Si papier n'est pas le plus probable
            del self.possibilite[self.possibilite.index("S")]  # Enlever Ciseaux (qui bat papier)
            
        # S'il reste plusieurs choix optimaux, en choisir un au hasard
        choice = [self.possibilite[0], self.possibilite[len(self.possibilite)-1]]
        self.possibilite = random.choice(choice)
        
        return self.possibilite

    # =======================================================================================================
    # BOUCLE PRINCIPALE DE JEU - Orchestration de toute la partie
    # =======================================================================================================

    def play(self):
        """
        Fonction principale qui gère le déroulement complet du jeu.
        
        Workflow:
            1. Initialisation (règles, points)
            2. Boucle de jeu principale:
               - Prédiction du coup avec Minasi
               - Annonce du décompte
               - Attente de détection du geste joueur
               - Comparaison et attribution des points
               - Annonce du résultat
               - Vérification fin de partie
            3. Fin de partie
        """
        
        # =======================================================================================================
        # INITIALISATION DE LA PARTIE
        # =======================================================================================================
        
        # Points de départ pour chaque joueur
        self.Robot_coins = 10
        self.Player_coins = 10

        # Explication des règles du jeu
        sleep(0.2)
        self.speak_this("Nous voici dans le jeu interactif pierre papier ciseaux avec détection de gestes.")
        sleep(6)
        self.speak_this("Tu dois simplement faire le geste avec ta main devant mes yeux.")
        sleep(4)
        self.speak_this("Je vais dire, Chi Fou Mi et Pierre, par exemple")
        sleep(5)
        self.speak_this("J'observerai ta main, et comparerai avec mon choix.")
        sleep(3)
        self.speak_this("Je ne triche jamais, mais suis assez doué à ce jeu, alors, bonne chance.")
        sleep(6)

        # Annonce des points de départ (phrase choisie aléatoirement)
        choice1 = ['On commence la partie avec 10 points chacun.',
                   'Nous avons tous les deux, dix points.',
                   'Toi et moi commençons la partie avec 10 points chacun.']
        self.speak_this(random.choice(choice1))
        sleep(3)
        self.speak_this("Tu es prêt; C'est parti.")
        sleep(4)

        # =======================================================================================================
        # BOUCLE PRINCIPALE DE JEU - Continue jusqu'à fin de partie
        # =======================================================================================================

        while rclpy.ok():  # Tant que ROS2 fonctionne (pas d'arrêt demandé)

            # =======================================================================================================
            # PHASE 1: PRÉDICTION DU ROBOT - Algorithme de Minasi
            # =======================================================================================================
            
            # Le robot prédit le prochain coup du joueur et choisit sa contre-attaque
            self.robot = self.getNextHit()

            # Conversion de la lettre du robot vers le nom du geste
            if self.robot == 'R':
                self.robot_coup = self.nom_des_formes[0]  # "pierre"
            if self.robot == 'P':
                self.robot_coup = self.nom_des_formes[1]  # "papier"
            if self.robot == 'S':
                self.robot_coup = self.nom_des_formes[2]  # "ciseaux"

            # =======================================================================================================
            # PHASE 2: RESET ET PRÉPARATION - Nettoyage des variables
            # =======================================================================================================
            
            # Reset des variables de détection pour ce tour
            self.coup_letter = ""  # Pas encore de coup détecté
            self.coup = ""         # Pas encore de nom de coup

            # =======================================================================================================
            # PHASE 3: DÉCOMPTE ET ANNONCE - Le robot lance le jeu
            # =======================================================================================================
            
            # Le robot annonce son choix selon le format traditionnel
            self.speak_this("chi; fou; mi")
            sleep(1.5)
            self.speak_this(self.robot_coup)

            # =======================================================================================================  
            # PHASE 4: ATTENTE DE DÉTECTION - Timeout de 1.5 secondes
            # =======================================================================================================
            
            # Attente de la détection du geste du joueur avec timeout court
            start_wait = time()
            while self.coup_letter == "" and (time() - start_wait) < 3:
                sleep(0.1)  # Petite pause pour ne pas surcharger le CPU
                # Traiter les messages ROS2 en attente (notamment les images)
                rclpy.spin_once(self, timeout_sec=0.01)

            # =======================================================================================================
            # PHASE 5: GESTION D'ABSENCE DE DÉTECTION - Timeout ou échec
            # =======================================================================================================
            
            # Si aucun geste n'a été détecté dans les temps
            if self.coup_letter == "":
                # Gestion des messages d'erreur avec variation selon les échecs répétés
                if self.bad_step >= 1:
                    # Messages pour échecs répétés (plus d'impatience)
                    nothing_seen = ["je ne vois toujours aucun geste "+self.genre, 
                                   "à nouveau, trop de temps", 
                                   "encore trop tard", 
                                   "toujours en retard", 
                                   "essaye de faire ton geste plus rapidement "+self.genre]
                    self.speak_this(random.choice(nothing_seen))
                    sleep(2)
                    self.bad_step = 0  # Reset du compteur
                else:
                    # Premier échec (messages plus polis)
                    nothing_seen = ["je ne détecte aucun geste. Nous devons donc recommencer", 
                                   "tu mets trop de temps, recommençons", 
                                   "oups, trop tard, réessayons"]
                    self.speak_this(random.choice(nothing_seen))
                    sleep(3)
                    self.bad_step += 1  # Incrément du compteur d'échecs
                
                # Recommencer ce tour (continue = retour au début de la boucle while)
                continue

            # =======================================================================================================
            # PHASE 6: ENREGISTREMENT DANS L'HISTORIQUE - Alimentation de Minasi
            # =======================================================================================================
            
            # Ajouter le coup du ROBOT à l'historique (majuscule)
            self.pattern += str(self.robot)

            # Ajouter le coup du JOUEUR à l'historique (minuscule)  
            self.pattern += self.coup_letter

            # Extraire la combinaison du dernier tour pour analyse
            self.dernier_coup = self.pattern[-2:]  # 2 derniers caractères: robot+joueur
            
            # =======================================================================================================
            # PHASE 7: PRÉPARATION DES NOMS - Pour l'annonce vocale
            # =======================================================================================================
            
            # Conversion des lettres en noms pour l'annonce (coup du ROBOT)
            if self.pattern[-2:][0] == 'R':  # Premier caractère = robot
                R = self.nom_des_formes[0]   # "pierre"
            if self.pattern[-2:][0] == 'P':
                R = self.nom_des_formes[1]   # "papier"
            if self.pattern[-2:][0] == 'S':
                R = self.nom_des_formes[2]   # "ciseaux"
                
            # Conversion des lettres en noms pour l'annonce (coup du JOUEUR)
            if self.pattern[-2:][1] == 'r':  # Deuxième caractère = joueur
                P = self.nom_des_formes[0]   # "pierre"
            if self.pattern[-2:][1] == 'p':
                P = self.nom_des_formes[1]   # "papier"
            if self.pattern[-2:][1] == 's':
                P = self.nom_des_formes[2]   # "ciseaux"

            # =======================================================================================================
            # PHASE 8: ANALYSE DU RÉSULTAT - Qui gagne ce tour?
            # =======================================================================================================

            # CAS 1: LE ROBOT GAGNE (le joueur perd)
            if self.dernier_coup in self.coups_gagnants:
                # Gestion des messages avec variation selon les victoires répétées
                if self.win >= random.randint(1,4):  # Après plusieurs victoires
                    self.speak_this(R+" contre "+P+random.choice(self.win_again_sentences))
                    sleep(3)
                    self.win = 0  # Reset du compteur
                else:
                    # Première victoire ou victoires espacées
                    self.speak_this(R+" contre "+P+random.choice(self.win_sentences))
                    sleep(3)
                    self.win += 1     # Incrément victoires consécutives
                    self.loose = 0    # Reset défaites consécutives
                    self.nul = 0      # Reset égalités consécutives
                    self.bad_step = 0 # Reset échecs de détection

                # Attribution des points: joueur gagne un point, robot en perd un
                self.Player_coins += 1
                self.Robot_coins -= 1

            # CAS 2: ÉGALITÉ (même geste des deux côtés)
            elif self.dernier_coup in self.coups_nuls:
                # Gestion des messages d'égalité avec variation
                if self.nul >= 1:  # Égalités répétées
                    self.speak_this(random.choice(self.nul_again_sentences))
                    sleep(3)
                    self.nul = 0  # Reset du compteur
                else:
                    # Première égalité
                    self.speak_this(random.choice(self.nul_sentences))
                    sleep(3)
                    self.nul += 1     # Incrément égalités consécutives
                    self.win = 0      # Reset victoires consécutives
                    self.loose = 0    # Reset défaites consécutives
                    self.bad_step = 0 # Reset échecs de détection
                
                # Pas de changement de points en cas d'égalité

            # CAS 3: LE ROBOT PERD (le joueur gagne)
            else:
                # Gestion des messages de défaite avec variation
                if self.loose >= random.randint(1,4):  # Après plusieurs défaites
                    self.speak_this(R+" contre "+P+random.choice(self.loose_again_sentences))
                    sleep(3)
                    self.loose = 0  # Reset du compteur
                else:
                    # Première défaite ou défaites espacées
                    self.speak_this(R+" contre "+P+random.choice(self.loose_sentences))
                    sleep(3)
                    self.loose += 1   # Incrément défaites consécutives
                    self.win = 0      # Reset victoires consécutives
                    self.nul = 0      # Reset égalités consécutives
                    self.bad_step = 0 # Reset échecs de détection

                # Attribution des points: robot gagne un point, joueur en perd un
                self.Robot_coins += 1
                self.Player_coins -= 1

            # =======================================================================================================
            # PHASE 9: VÉRIFICATION FIN DE PARTIE - Un joueur à 0 points?
            # =======================================================================================================

            # Le robot n'a plus de points = le joueur a gagné la partie
            if self.Robot_coins == 0:
                self.speak_this("Je me retrouve à zéro points. Tu as gagné la partie, bravo.")
                sleep(4)
                self.speak_this(random.choice(self.stop_sentences))
                sleep(3)
                break  # Sortir de la boucle de jeu

            # Le joueur n'a plus de points = le robot a gagné la partie
            if self.Player_coins == 0:
                self.speak_this("Tu n'as plus de points. j'ai gagné, cool.")
                sleep(3)
                self.speak_this(random.choice(self.stop_sentences))
                sleep(3)
                break  # Sortir de la boucle de jeu

            # =======================================================================================================
            # PHASE 10: ANNONCE DES SCORES - Seulement si pas d'égalité
            # =======================================================================================================

            # Log des scores pour debug/monitoring
            self.get_logger().info(f'Robot_coins: {self.Robot_coins}')
            self.get_logger().info(f'Player_coins: {self.Player_coins}')

            # Annoncer les scores seulement s'il y a eu changement (pas d'égalité)
            if not self.dernier_coup in self.coups_nuls:
                # Différentes façons d'annoncer le score (pour la variété)
                result = ["j'ai "+str(self.Robot_coins)+" points, et toi "+str(self.Player_coins), 
                         "tu as "+str(self.Player_coins)+" points, et moi "+str(self.Robot_coins), 
                         str(self.Robot_coins)+" à "+str(self.Player_coins), 
                         str(self.Robot_coins)+" contre "+str(self.Player_coins)]
                self.speak_this(random.choice(result))
                sleep(3)

            # =======================================================================================================
            # PHASE 11: PAUSE ENTRE LES TOURS - Tempo naturel
            # =======================================================================================================
            
            # Pause aléatoire avant le prochain tour (rend le jeu plus naturel)
            sleep(random.uniform(0.1, 1.5))


# =======================================================================================================
# FONCTION PRINCIPALE - Point d'entrée du programme
# =======================================================================================================

def main(args=None):
    """
    Fonction principale qui initialise et lance le jeu Neo CHIFUMI.
    
    Args:
        args: Arguments de ligne de commande (optionnel)
        
    Workflow:
        1. Initialisation de ROS2
        2. Création du nœud de jeu
        3. Lancement de la partie
        4. Gestion propre de l'arrêt
    """
    
    # Initialisation du système ROS2
    rclpy.init(args=args)
    
    # Création de l'instance du nœud de jeu
    game_node = GameNode()
    
    try:
        # Lancement de la partie (fonction bloquante jusqu'à fin de partie)
        game_node.play()
        
    except KeyboardInterrupt:
        # Gestion de l'interruption clavier (Ctrl+C)
        game_node.get_logger().info('Game interrupted by user')
        
    finally:
        # Nettoyage propre à la fin
        game_node.destroy_node()  # Destruction du nœud ROS2
        rclpy.shutdown()          # Arrêt de ROS2


# =======================================================================================================
# POINT D'ENTRÉE - Exécution directe du script
# =======================================================================================================

if __name__ == "__main__":
    """
    Point d'entrée du programme.
    Appelé automatiquement quand le script est exécuté directement
    (pas quand il est importé comme module).
    """
    main()  