# ✅ Objectif : Lancer startup.launch.py automatiquement au démarrage

Créer un service systemd qui :

    - Lance automatiquement ros2 launch mon_package startup.launch.py au boot.

    - S’exécute avec l’utilisateur (non root).

    - Log les sorties via journalctl.

    - Redémarre en cas d’erreur.

    - Est activé automatiquement au démarrage.

## 🧾 1. Crée un service systemd

```bash
sudo nano /etc/systemd/system/qbo_bringup.service
```

## 🧱 2. Contenu du fichier qbo_bringup.service

```ini
[Unit]
Description=Qbo ROS 2 Bringup
After=network.target

[Service]
User=qbo-v2
WorkingDirectory=/home/qbo-v2/qbo_ws/
Environment="HOME=/home/qbo-v2"
Environment="DISPLAY=:0.0"
ExecStart=/bin/bash -c "source /opt/ros/humble/setup.bash && source /home/qbo-v2/qbo_ws/install/setup.bash && ros2 launch qbo_bringup qbo_startup.launch.py"
Restart=on-failure
RestartSec=5
StandardOutput=journal
StandardError=journal

[Install]
WantedBy=multi-user.target
```

## ⚙️ 3. Activer le service au boot

```bash
sudo systemctl daemon-reload
sudo systemctl enable qbo_bringup.service
sudo systemctl start qbo_bringup.service
```

## 🔍 4. Suivre les logs

```bash
# Voir l’état du service
systemctl status qbo_bringup.service

# Voir les logs récents
journalctl -u qbo_bringup.service

# Suivre en direct
journalctl -fu qbo_bringup.service
```

## 🧪 5. Vérification au reboot
Redémarre, puis dès le démarrage, connecte-toi et vérifie :
```bash
systemctl status qbo_bringup.service
```

# 🔧 Commandes principales

## ▶️ Lancer manuellement le service
```bash
sudo systemctl start qbo_bringup.service
```

## ⏹️ Arrêter le service
```bash
sudo systemctl stop qbo_bringup.service
```

## 🔁 Redémarrer le service
```bash
sudo systemctl restart qbo_bringup.service
```

## ✅ Activer au boot
```bash
sudo systemctl enable qbo_bringup.service
```

## ❌ Désactiver au boot
```bash
sudo systemctl disable qbo_bringup.service
```

## 🛠️ En cas de modification du fichier .service
Si tu modifies le fichier /etc/systemd/system/qbo_bringup.service, recharge systemd :
```bash
sudo systemctl daemon-reload
```
Puis redémarre le service :
```bash
sudo systemctl restart qbo_bringup.service
```
