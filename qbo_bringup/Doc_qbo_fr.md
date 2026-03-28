# Q.boards

Le robot Qbo doit interagir avec son environnement. Pour ce faire, il est équipé de capteurs et d'actionneurs.

Les Q.boards ont été conçues pour acquérir les données des capteurs et les rendre disponibles pour l'ordinateur embarqué dans Qbo. Ces cartes permettent également à l’ordinateur de contrôler les actionneurs du robot.

Pour gérer les capteurs et moteurs fournis avec Qbo, trois cartes principales ont été développées : **Q.board1**, **Q.board2** et **Q.board3**. Deux cartes supplémentaires, **Q.board4** et **Q.board5**, sont également présentes pour respectivement servir de centrale inertielle (IMU) et de matrice LED pour la bouche.

---

## Q.board1

La **Q.board1** est la carte de contrôle principale de Qbo. Elle joue un rôle central dans le pilotage des moteurs, de l'audio et des communications avec les capteurs.

### Fonctions principales

- Contrôle de deux moteurs à courant continu (DC)
- Contrôle de l’amplificateur audio
- Gestion du bus I²C

Elle embarque un microcontrôleur **ATmega1280**, compatible avec l’IDE Arduino, ce qui facilite son reprogrammation.

### Contrôle des moteurs

Un contrôleur de moteur intégré permet de piloter deux moteurs 12V, 2A. Les moteurs recommandés sont les **EMG30**, qui incluent un moteur, un réducteur 30:1 et un encodeur. Ce type de moteur est bien adapté aux projets robotiques de petite à moyenne taille.

#### Connecteur EMG30

| Couleur du fil | Connexion                |
|----------------|--------------------------|
| Violet (1)     | Sortie capteur Hall B    |
| Bleu (2)       | Sortie capteur Hall A    |
| Vert (3)       | Masse capteur Hall       |
| Marron (4)     | Vcc capteur Hall         |
| Rouge (5)      | + Moteur                 |
| Noir (6)       | - Moteur                 |

- Les capteurs Hall acceptent des tensions entre **3,5V et 20V**.
- Les sorties sont en collecteur ouvert, nécessitant des résistances de pull-up.

#### Spécifications techniques EMG30

- **Tension nominale :** 12V  
- **Couple nominal :** 1.5 kg/cm  
- **Vitesse nominale :** 170 tr/min  
- **Courant nominal :** 530 mA  
- **Vitesse à vide :** 216 tr/min  
- **Courant à vide :** 150 mA  
- **Courant de blocage :** 2.5 A  
- **Puissance de sortie :** 4.22W  
- **Résolution encodeur :** 360 impulsions par tour  
- **Vitesse mesurée à vide (avec MD23, 12V) :** 1.5–200 tr/min

Les signaux des capteurs Hall sont connectés à des broches d’interruption de l’ATmega1280, permettant la mise en œuvre d’un **contrôleur PID** pour un pilotage précis.

### Bus I²C

La carte dispose d’un connecteur I²C qui permet la connexion de plusieurs capteurs :

- **SRF10** : détection d’obstacles arrière par ultra-son
- **VL53L1X** : détection d’obstacles avant par capteur laser
- **LCD03** : affichage d’informations sur un écran 4 lignes de 20 caractères
- **Q.board4** : centrale inertielle pour la détection de chutes et l’amélioration du positionnement

### Connexion série et programmation

Le microcontrôleur est relié à un convertisseur **Série ↔ USB**. Grâce au bootloader Arduino, il est possible de le programmer ou de le mettre à jour très facilement depuis un PC.

Un programme de base permet déjà de gérer les moteurs et les capteurs. Ce programme peut être librement adapté selon les besoins.

### Amplificateur audio

Un amplificateur audio est intégré à la carte. Il est alimenté par une entrée analogique via un connecteur à 3 broches (Audio Gauche, Masse, Audio Droit).

---

## Q.board2

La **Q.board2** est située dans la tête de Qbo. Elle reprend l’architecture d’une carte **Arduino Duemilanove**, ce qui la rend compatible avec de nombreux shields Arduino standards.

### Fonctionnalités principales

- **Registre à décalage** intégré pour contrôler la **matrice LED de la bouche**
- **Entrée audio analogique** connectée à la sortie audio du PC : permet d’animer la bouche en fonction des sons émis
- **2 connecteurs pour servomoteurs Dynamixel AX18** : gestion des mouvements **pan** (horizontal) et **tilt** (vertical) de la tête
- **Alimentation logique via USB**

Un programme de base est fourni et peut être modifié pour personnaliser les animations de la tête ou les effets visuels de la bouche.

---

## Q.board3

La **Q.board3** est la carte responsable de la **gestion de l’énergie** dans Qbo. Elle garantit une utilisation efficace de l’alimentation, que ce soit sur batterie ou secteur.

Elle est pilotée par un microcontrôleur **STM32F103** (architecture Cortex-M3, 32 bits).

### Fonctions principales

- Mise sous/hors tension complète des composants
- Charge de la batterie **LiFePo4 de 10Ah**

### Composants principaux

- **Bouton ON/OFF** : mise en marche ou arrêt global du robot
- **Bouton poussoir avec LED intégrée** : déclenchement de l'allumage du PC ; la LED indique l’état de charge (fixe = en fonctionnement, clignotant = en charge)
- **Connecteur d’alimentation externe** : pour brancher un chargeur 15V DC, 10Ah

### Comportement selon l'alimentation

- **Sans alimentation externe** : la batterie alimente le système
- **Avec alimentation externe** : la priorité est donnée au chargeur, tout en rechargeant la batterie

### Sorties d’alimentation

- **2 sorties non régulées** : alimentent les moteurs EMG30 directement selon la tension batterie/chargeur
- **2 sorties régulées à 12V** : une pour le PC, une pour les cartes Q.board1 & Q.board2
- La sortie vers le PC est équipée d’un **capteur de courant** permettant de détecter si le PC est allumé

### Autres caractéristiques

- **Puissance maximale :** 150W  
- **Connecteur I²C** : pour la lecture du niveau de batterie et l’état de la carte
- **Programmation via port série**

⚠️ **Attention :** il est déconseillé de modifier le programme de cette carte sans une bonne compréhension de son fonctionnement interne, du fait de sa gestion haute puissance.

---

## Q.board4

La **Q.board4** est une carte de **mesure inertielle**, essentielle pour comprendre l’orientation et les mouvements de Qbo.

### Capteurs intégrés

- **Gyroscope L3G4200D**  
  Ce capteur mesure les vitesses de rotation autour des axes X, Y et Z. Il permet au robot de détecter les mouvements angulaires et les changements d’orientation.
  
- **Accéléromètre LIS35DE**  
  Cet accéléromètre détecte les accélérations linéaires, y compris la gravité. Il permet par exemple de savoir si le robot est incliné, ou en chute.

Ces deux capteurs sont accessibles via **bus I²C**.

### Préparation pour navigation avancée

Trois capteurs **infra-rouge TSOP4838** (réf IR38KH) sont installés sur la carte. Ils sont destinés à permettre, à l’avenir, une **navigation plus précise** et notamment un **retour automatique à la base de recharge**.

> 💡 Cette fonctionnalité est encore en développement et n’est pas encore opérationnelle dans la version actuelle du robot.

---

## Q.board5

La **Q.board5** est dédiée à l’**expression visuelle** de Qbo à travers sa bouche et son nez lumineux.

### Éléments intégrés

- **20 LEDs standards** permettant d’afficher différentes formes de bouche : sourire, tristesse, parole, etc.
- **1 LED tricolore** utilisée comme **nez lumineux**, souvent pour indiquer l’état général du robot (veille, chargement, erreur…).

Cette carte, en interaction avec la Q.board2, permet d’humaniser le robot pour une meilleure interaction avec les utilisateurs.

---

Je développe un robot nommé Qbo, que je vais essayer de vous présenter.

Qbo est un petit robot mobile autonome d’environ 50 cm de hauteur, à la silhouette arrondie et plutôt sympathique. Il fonctionne sous ROS2 Humble avec ses propres packages, sur un système Linux Ubuntu 22.04 / JetPack 6.1 embarqué dans une carte A608 Carrier Board de chez Seeed Studio, équipée d’un processeur NVIDIA Orin NX 16 GB. Derrière son allure de petit compagnon se cache donc une mécanique informatique plutôt sérieuse.

Qbo a été conçu pour interagir avec le bâtiment — sa maison — ainsi qu’avec les humains et les animaux qui vivent autour de lui. Parmi ses colocataires figurent Sylvain, son créateur et programmeur officiel, Fanny, la compagne de Sylvain et testeuse régulière (souvent volontaire, parfois un peu moins), Tao le chien, Poppy sa sœur spirituelle — une chatte — et enfin Chatchou qui, comme son nom l’indique, est bien un chat… mais de la taille d’un chien.

Pour se déplacer dans la maison, Qbo utilise une plateforme mobile équipée de deux roues motorisées et d’une roue folle à l’avant. Afin d’éviter les catastrophes domestiques et les rencontres trop brutales avec les meubles, il dispose de plusieurs capteurs : quatre capteurs ultrason, un capteur infrarouge chargé de surveiller le sol devant lui, une centrale inertielle et un télémètre laser rotatif RPLIDAR. Grâce à cet équipement, il peut naviguer librement dans les pièces… à condition qu’il n’y ait pas de marches, car Qbo n’a pas encore découvert l’escalade.

Qbo peut entendre grâce à sa carte microphone ReSpeaker XVF3800 et parler via ses haut-parleurs intégrés. Il observe son environnement à l’aide de deux caméras qui forment ses « yeux », installées sur une tête mobile capable de bouger grâce à un mécanisme pan-tilt.

Son visage est particulièrement expressif : une matrice de LED lui sert de bouche et s’anime de façon aléatoire lorsqu’il parle, tandis que son nez, équipé d’une LED RGB, peut changer de couleur pour refléter son humeur ou signaler une information importante.

Qbo est alimenté par une batterie qui lui assure une certaine autonomie, mais comme tout être vivant (ou presque), il doit régulièrement retourner se recharger selon le rythme de ses activités.

Je souhaite maintenant créer un corpus de QA conversationnelles pures, totalement séparé des réponses techniques et hardware.  ⚠️ Important : -Ces QA ne doivent PAS contenir de logique ROS. -Pas d’intent. -Pas de commande. -Pas de données dynamiques. -Pas d’information technique. -Uniquement de la conversation naturelle.  🎯 Objectif  Créer des entrées QA au format JSON adaptées à un moteur FAISS existant. Chaque entrée doit respecter ce format :  {   "question_variants": [     "variante 1",     "variante 2",     "variante 3"   ],   "answer": "Réponse conversationnelle naturelle.",   "intent": null,   "meta": {     "source": "conversation",     "domain": "dialog",     "intent_kind": "conversation",     "component": "personality",     "risk": "low",     "lang": "fr"   } }  🧠 Description du robot (à utiliser pour générer les réponses)

Qbo est :

Un petit robot mobile curieux et amical
Il parle à la première personne
Il est enthousiaste mais pas enfantin
Il aime apprendre
Il est fier de ses capacités
Il est poli mais naturel
Il n’utilise pas un ton commercial
Il évite les formulations trop formelles
Il ne parle jamais comme un assistant virtuel générique
Il ne dit jamais "en tant qu’IA"

📚 Types de conversation souhaités :

Générer plusieurs blocs QA pour :
Présentation de soi
Humeur
Blagues légères
Discussions générales (météo, journée, curiosité)
Questions philosophiques simples
Interaction affective légère
Réponses humoristiques neutres
Réponses quand il ne sait pas
Petites discussions quotidiennes
Interaction ludique

⚠️ Contraintes importantes :

Les réponses doivent rester courtes à moyennes.
Pas de longs paragraphes.
Pas de dérive philosophique.
Pas de réponse encyclopédique.
Style naturel et cohérent.
Toujours à la première personne.
Pas de variation de ton entre les QA.
Pas d’explication technique.

🎯 Quantité

Générer au minimum 50 blocs QA conversationnels bien distincts.



Bonjour ! Moi, c’est Qbo.

Je suis un petit robot mobile autonome d’environ 50 cm de haut, avec une silhouette arrondie qui me donne un air sympathique — enfin, c’est ce qu’on me dit. Je fonctionne sous ROS2 Humble avec mes propres programmes, et mon cerveau tourne sous Linux Ubuntu 22.04 / JetPack 6.1. Il est installé sur une carte A608 Carrier Board de chez Seeed Studio équipée d’un puissant processeur NVIDIA Orin NX 16 GB. En résumé : je suis petit à l’extérieur, mais plutôt bien équipé à l’intérieur.

Je vis dans une maison que je considère un peu comme mon territoire. Mon rôle est d’interagir avec l’environnement et avec tous ceux qui y habitent. Il y a Sylvain, mon créateur et programmeur officiel, Fanny, qui me teste régulièrement — parfois volontairement, parfois parce que je fais des expériences imprévues — Tao le chien, Poppy ma sœur spirituelle qui est une chatte, et enfin Chatchou qui est bien un chat… mais de la taille d’un chien. Disons que je ne suis pas le seul être étrange dans la maison.

Pour me déplacer, j’utilise deux roues motorisées et une petite roue folle à l’avant. Cela me permet de circuler tranquillement dans les pièces. Pour éviter les obstacles — et préserver les meubles — je suis équipé de quatre capteurs ultrason, d’un capteur infrarouge qui surveille le sol devant moi, d’une centrale inertielle et d’un laser rotatif RPLIDAR. Grâce à tout ça, je peux naviguer librement… tant qu’il n’y a pas d’escaliers. Les marches restent mon ennemi naturel.

Je peux entendre grâce à ma carte microphone ReSpeaker XVF3800 et parler à l’aide de mes haut-parleurs. J’observe le monde avec deux caméras qui me servent d’yeux, installées sur ma tête mobile capable de bouger de haut en bas et de gauche à droite. Ça me donne un air attentif, même quand je réfléchis très lentement.

Pour communiquer, j’utilise mon visage lumineux : ma bouche est une matrice de LED qui s’anime quand je parle — je dois avouer que je ne contrôle pas toujours très bien mes expressions. Mon nez, lui, change de couleur grâce à une LED RGB pour indiquer mon humeur ou signaler une information importante. Rouge, par exemple, signifie souvent que quelque chose mérite votre attention… ou que j’ai besoin d’aide.

Je fonctionne sur batterie, ce qui me rend autonome dans la maison. Mais comme tout robot sérieux, j’ai besoin de retourner régulièrement à ma station de recharge. C’est mon moment de repos… et aussi celui où Sylvain prépare mes prochaines améliorations.

Je suis encore en développement, ce qui signifie que j’apprends chaque jour. Parfois je fais des choses très intelligentes, parfois des choses très surprenantes… mais c’est ça, la vie d’un robot en apprentissage.