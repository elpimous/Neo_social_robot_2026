#!/bin/bash
# =============================================================================
#  install_network.sh — connexion Wi-Fi fiable pour Néo (à lancer UNE fois)
#  Auteur : Vincent FOUCAULT
#  Usage : sudo bash install_network.sh
# =============================================================================
set -e
CON="Freebox-A39FDE"
HERE="$(cd "$(dirname "$0")" && pwd)"
[ "$(id -u)" -eq 0 ] || { echo "Lance avec sudo"; exit 1; }

echo "1/6  Économie d'énergie Wi-Fi désactivée (définitivement)"
mkdir -p /etc/NetworkManager/conf.d
cat > /etc/NetworkManager/conf.d/neo-wifi-powersave-off.conf <<'CONF'
[connection]
wifi.powersave = 2
CONF
nmcli connection modify "$CON" 802-11-wireless.powersave 2 || true

echo "2/6  Reconnexion automatique illimitée"
nmcli connection modify "$CON" connection.autoconnect yes \
      connection.autoconnect-retries 0 connection.autoconnect-priority 100 || true
# pas de changement d'adresse MAC aléatoire (la box doit toujours reconnaître Néo)
nmcli connection modify "$CON" 802-11-wireless.mac-address-randomization never || true

echo "3/6  Journal système conservé entre redémarrages"
mkdir -p /var/log/journal
sed -i 's/^#\?Storage=.*/Storage=persistent/' /etc/systemd/journald.conf
systemctl restart systemd-journald

echo "4/6  Gardien Wi-Fi"
install -m 755 "$HERE/neo_netguard.sh" /usr/local/bin/neo_netguard.sh
install -m 644 "$HERE/neo-netguard.service" /etc/systemd/system/neo-netguard.service
systemctl daemon-reload
systemctl enable --now neo-netguard.service

echo "5/6  SSH : détection rapide des connexions mortes"
cat > /etc/ssh/sshd_config.d/neo-keepalive.conf <<'CONF'
ClientAliveInterval 15
ClientAliveCountMax 4
TCPKeepAlive yes
CONF
systemctl reload ssh || systemctl reload sshd || true

echo "6/6  Application"
systemctl reload NetworkManager
DEV=$(nmcli -t -f DEVICE,TYPE device | awk -F: '$2=="wifi"{print $1; exit}')
iw dev "$DEV" set power_save off 2>/dev/null || true
echo
echo "Économie d'énergie : $(iw dev "$DEV" get power_save 2>/dev/null)"
systemctl --no-pager --lines=0 status neo-netguard.service | head -3
echo "Journal du gardien : tail -f /var/log/neo_netguard.log"
