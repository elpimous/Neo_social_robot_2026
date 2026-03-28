# ✅ Qbo ROS 2 Bringup — systemd & timer setup

This guide explains how to automatically launch `qbo_startup.launch.py` using `systemd` with a post-boot delay via a `.timer`.

---

## ✅ Goals

- 🔁 Automatically start the bringup 20 seconds after boot
- 👤 Run as non-root user (`qbo-v2`)
- 📋 Log output with `journalctl`
- 🔄 Auto-restart on failure
- 🕓 Use a systemd `.timer` for robust post-boot triggering

---

## 🧾 1. Create the service file

```bash
sudo nano /etc/systemd/system/qbo_bringup.service
```

### Contents:

```ini
[Unit]
Description=Qbo ROS 2 Bringup
After=network-online.target jtop.service
Requires=network-online.target jtop.service

StartLimitIntervalSec=0

[Service]
Type=simple
User=qbo-v2
WorkingDirectory=/home/qbo-v2/qbo_ws/
Environment="HOME=/home/qbo-v2"
Environment="DISPLAY=:0.0"

# 🧠 Lancement ROS2
ExecStart=/bin/bash -c "echo 'STARTING QBO BRINGUP' >> /tmp/qbo_bringup_debug.log && date >> /tmp/qbo_bringup_debug.log && source /opt/ros/humble/setup.bash && source /home/qbo-v2/qbo_ws/install/setup.bash && ros2 launch qbo_bringup qbo_startup.launch.py"

Restart=on-failure
RestartSec=10
StandardOutput=journal
StandardError=journal

[Install]
WantedBy=none
```

---

## ⏲️ 2. Create the timer file

```bash
sudo nano /etc/systemd/system/qbo_bringup.timer
```

### Contents:

```ini
[Unit]
Description=Start Qbo Bringup after boot delay

[Timer]
OnBootSec=20
Unit=qbo_bringup.service

[Install]
WantedBy=timers.target
```

---

## ⚙️ 3. Enable and start the timer

```bash
sudo systemctl daemon-reload
sudo systemctl enable qbo_bringup.timer
sudo systemctl start qbo_bringup.timer
```

---

## 🔍 4. Monitoring

### Timer status:

```bash
systemctl list-timers | grep qbo
systemctl status qbo_bringup.timer
```

### Service status and logs:

```bash
systemctl status qbo_bringup.service
journalctl -u qbo_bringup.service
journalctl -fu qbo_bringup.service
```

### Debug log output:

```bash
cat /tmp/qbo_bringup_debug.log
```

---

## 🧪 5. Post-reboot check

After reboot, wait ~20s then run:

```bash
journalctl -u qbo_bringup.service
```

Or:

```bash
cat /tmp/qbo_bringup_debug.log
```

---

# 🔧 Systemd Commands

## Service

```bash
sudo systemctl start qbo_bringup.service
sudo systemctl stop qbo_bringup.service
sudo systemctl restart qbo_bringup.service
```

## Timer

```bash
sudo systemctl start qbo_bringup.timer
sudo systemctl enable qbo_bringup.timer
sudo systemctl stop qbo_bringup.timer
sudo systemctl disable qbo_bringup.timer
```

---

## 🛠 When modifying the service or timer files

```bash
sudo systemctl daemon-reexec
sudo systemctl daemon-reload
sudo systemctl restart qbo_bringup.timer
```
