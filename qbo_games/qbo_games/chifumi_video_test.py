#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Simple Chifoumi Hand Gesture Detector - chifumi_video_test.py
Détecte pierre/papier/ciseaux et publie le flux vidéo annoté
Date: 02 Août 2025
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import mediapipe as mp


class ChifoumiVideoTest(Node):

    def __init__(self):
        super().__init__('chifumi_video_test_node')
        
        # MediaPipe setup
        self.mp_hands = mp.solutions.hands
        self.hands = self.mp_hands.Hands(
            static_image_mode=False,
            max_num_hands=1,
            min_detection_confidence=0.7,
            min_tracking_confidence=0.5,
            model_complexity=0
        )
        self.mp_drawing = mp.solutions.drawing_utils
        
        # CV Bridge
        self.cv_bridge = CvBridge()
        
        # FPS calculation
        self.fps_counter = 0
        self.fps_start_time = None
        self.current_fps = 0.0
        
        # ROS2 Subscribers and Publishers
        self.image_subscription = self.create_subscription(
            Image,
            '/inverted_eye_image',
            self.image_callback,
            10)
            
        self.debug_publisher = self.create_publisher(
            Image,
            '/chifoumi_debug',
            10)
        
        self.get_logger().info('Chifumi Video Test initialized')

    def classify_gesture(self, landmarks):
        """Classifie le geste et retourne (nom, confiance, total_fingers)"""
        
        # Récupère les positions des landmarks
        landmark_list = []
        for lm in landmarks.landmark:
            landmark_list.append([lm.x, lm.y])
        
        # Indices des articulations des doigts
        # Pouce: 4 (bout), 3 (articulation)
        # Index: 8 (bout), 6 (articulation) 
        # Majeur: 12 (bout), 10 (articulation)
        # Annulaire: 16 (bout), 14 (articulation)
        # Auriculaire: 20 (bout), 18 (articulation)
        
        fingers_up = []
        
        # Pouce (cas spécial - compare les coordonnées x)
        if landmark_list[4][0] > landmark_list[3][0]:  # Main droite
            fingers_up.append(1)
        else:
            fingers_up.append(0)
            
        # Autres doigts (compare les coordonnées y)
        finger_tips = [8, 12, 16, 20]
        finger_pips = [6, 10, 14, 18]
        
        for tip, pip in zip(finger_tips, finger_pips):
            if landmark_list[tip][1] < landmark_list[pip][1]:
                fingers_up.append(1)
            else:
                fingers_up.append(0)
        
        total_fingers = sum(fingers_up)
        
        # Classification avec confiance basique
        if total_fingers <= 1:
            return "PIERRE", 0.9, total_fingers
        elif 3 < total_fingers < 6:
            return "PAPIER", 0.9, total_fingers
        elif 1 < total_fingers < 4 and fingers_up[1] == 1 and fingers_up[2] == 1:
            return "CISEAUX", 0.9, total_fingers
        
        return "INCONNU", 0.3, total_fingers

    def image_callback(self, msg):
        try:
            # Calcul des FPS
            import time
            current_time = time.time()
            if self.fps_start_time is None:
                self.fps_start_time = current_time
                self.fps_counter = 0
            
            self.fps_counter += 1
            elapsed_time = current_time - self.fps_start_time
            
            if elapsed_time >= 1.0:  # Mise à jour des FPS chaque seconde
                self.current_fps = self.fps_counter / elapsed_time
                self.fps_counter = 0
                self.fps_start_time = current_time
            
            # Conversion ROS2 vers OpenCV
            cv_image = self.cv_bridge.imgmsg_to_cv2(msg, "bgr8")
            height, width = cv_image.shape[:2]
            
            # Conversion BGR vers RGB pour MediaPipe
            rgb_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)
            
            # Traitement avec MediaPipe
            results = self.hands.process(rgb_image)
            
            # Image de debug
            debug_image = cv_image.copy()
            
            gesture_name = "AUCUNE MAIN"
            confidence = 0.0
            total_fingers = 0
            
            # Analyse des landmarks
            if results.multi_hand_landmarks:
                for hand_landmarks in results.multi_hand_landmarks:
                    # Dessine les landmarks
                    self.mp_drawing.draw_landmarks(
                        debug_image, hand_landmarks, self.mp_hands.HAND_CONNECTIONS)
                    
                    # Classification du geste
                    gesture_name, confidence, total_fingers = self.classify_gesture(hand_landmarks)
            
            # Affichage du texte sur l'image
            font = cv2.FONT_HERSHEY_SIMPLEX
            font_scale = 1.0
            thickness = 2
            
            # Couleur selon la confiance
            if confidence > 0.8:
                color = (0, 255, 0)  # Vert
            elif confidence > 0.6:
                color = (0, 165, 255)  # Orange
            else:
                color = (0, 0, 255)  # Rouge
            
            # Texte principal
            text = f"{gesture_name}"
            cv2.putText(debug_image, text, (50, 50), font, font_scale, color, thickness)
            
            # Taux de confiance
            confidence_text = f"Confiance: {confidence:.1%}"
            cv2.putText(debug_image, confidence_text, (50, 90), font, 0.7, color, 2)
            
            # Total des doigts levés
            fingers_text = f"Up fingers: {total_fingers}"
            cv2.putText(debug_image, fingers_text, (50, 130), font, 0.7, (255, 255, 0), 2)
            
            # FPS (en haut à droite)
            fps_text = f"FPS: {self.current_fps:.1f}"
            cv2.putText(debug_image, fps_text, (width-120, 30), font, 0.6, (255, 255, 255), 2)
            
            # Publication de l'image debug
            debug_msg = self.cv_bridge.cv2_to_imgmsg(debug_image, "bgr8")
            debug_msg.header = msg.header
            self.debug_publisher.publish(debug_msg)
            
            # Log périodique
            if gesture_name != "AUCUNE MAIN":
                self.get_logger().info(f'Détection: {gesture_name} ({confidence:.1%}) - {total_fingers} doigts')
                
        except Exception as e:
            self.get_logger().error(f'Erreur dans image_callback: {e}')


def main(args=None):
    rclpy.init(args=args)
    
    detector = ChifoumiVideoTest()
    
    try:
        rclpy.spin(detector)
    except KeyboardInterrupt:
        detector.get_logger().info('Arrêt du détecteur vidéo')
    finally:
        detector.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()