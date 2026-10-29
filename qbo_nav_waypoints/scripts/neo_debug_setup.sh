#!/bin/bash
# =============================================================================
#  neo_debug_setup.sh — préparation du Jetson pour diagnostiquer les plantages
#  Auteur : Vincent FOUCAULT          Usage :  sudo bash neo_debug_setup.sh
# =============================================================================
set -e

echo "1) Journal système PERSISTANT (logs conservés après un redémarrage)"
mkdir -p /var/log/journal
systemd-tmpfiles --create --prefix /var/log/journal
systemctl restart systemd-journald

echo "2) Wi-Fi : désactivation de l'économie d'énergie (cause classique de perte SSH sur Jetson)"
for IF in $(ls /sys/class/net | grep -E '^wl'); do
  iw dev "$IF" set power_save off && echo "   $IF : power_save off"
done
mkdir -p /etc/NetworkManager/conf.d
printf "[connection]\nwifi.powersave = 2\n" > /etc/NetworkManager/conf.d/99-neo-wifi-powersave-off.conf
echo "   (permanent via NetworkManager : wifi.powersave = 2)"

echo "3) dmesg lisible sans root (pour la boîte noire neo_monitor)"
sysctl -w kernel.dmesg_restrict=0
echo "kernel.dmesg_restrict=0" > /etc/sysctl.d/99-neo-dmesg.conf

echo "4) Mode puissance du Jetson"
nvpmodel -q 2>/dev/null | head -2 || true
echo "   -> pour la pleine puissance : sudo nvpmodel -m 0 && sudo jetson_clocks"

echo
echo "OK. Après un plantage, au redémarrage :"
echo "   uptime                                  # quelques minutes = le Jetson a REDÉMARRÉ"
echo "   tail -60 \$(ls -t ~/.ros/qbo_nav_test/blackbox_*.log | sed -n 2p)   # boîte noire de la session plantée"
echo "   journalctl -b -1 -p warning --no-pager | tail -50"
