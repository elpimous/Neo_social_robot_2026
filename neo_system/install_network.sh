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

echo "1b   Pilote Realtek : économie d'énergie coupée au niveau du pilote"
DRV=$(basename "$(readlink /sys/class/net/$(nmcli -t -f DEVICE,TYPE device | awk -F: '$2=="wifi"{print $1; exit}')/device/driver/module 2>/dev/null)")
if [ -n "$DRV" ]; then
  OPTS=""
  for p in rtw_power_mgnt=0 rtw_ips_mode=0 rtw_lps_level=0; do
    modinfo "$DRV" 2>/dev/null | grep -q "parm: *${p%%=*}:" && OPTS="$OPTS $p"
  done
  if [ -n "$OPTS" ]; then
    echo "options $DRV$OPTS" > /etc/modprobe.d/neo-wifi.conf
    echo "     /etc/modprobe.d/neo-wifi.conf : options $DRV$OPTS (actif au prochain démarrage)"
  fi
fi

echo "1c   Bande 2.4 GHz imposée + IP fixe (le Wi-Fi était repassé en 5 GHz canal 60 DFS)"
nmcli connection modify "$CON" 802-11-wireless.band bg 802-11-wireless.channel 0 \
      ipv4.method manual ipv4.addresses 192.168.1.23/24 ipv4.gateway 192.168.1.254 \
      ipv4.dns 192.168.1.254 ipv6.method ignore
# aucun autre profil Wi-Fi ne doit pouvoir passer devant
for c in $(nmcli -t -f NAME,TYPE connection show | awk -F: '$2=="802-11-wireless"{print $1}'); do
  [ "$c" != "$CON" ] && nmcli connection modify "$c" connection.autoconnect no && echo "     profil '$c' : autoconnect désactivé"
done

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

echo "6/6  Application (coupure Wi-Fi de ~10 s : le SSH peut décrocher, c'est normal)"
systemctl reload NetworkManager
sleep 2
nmcli connection up "$CON" || true
sleep 5
DEV=$(nmcli -t -f DEVICE,TYPE device | awk -F: '$2=="wifi"{print $1; exit}')
iw dev "$DEV" set power_save off 2>/dev/null || true
echo
echo "Économie d'énergie : $(iw dev "$DEV" get power_save 2>/dev/null)"
iw dev "$DEV" info | grep -E "ssid|channel"
systemctl --no-pager --lines=0 status neo-netguard.service | head -3
echo "Journal du gardien : tail -f /var/log/neo_netguard.log"
