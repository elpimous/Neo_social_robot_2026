#!/bin/bash
# =============================================================================
#  neo_netguard.sh — gardien du Wi-Fi de Néo (service systemd neo-netguard)
#  Auteur : Vincent FOUCAULT
#
#  - toutes les 5 s : ping de la box (passerelle)
#  - toutes les 30 s : une ligne de journal (signal, débit, fréquence, ping,
#    charge CPU, température) => après une coupure on sait CE QUI s'est passé
#  - en cas de perte, réparation graduée, SANS jamais redémarrer la Jetson :
#      15 s  : relance de la connexion NetworkManager
#      40 s  : coupure / remise de la radio Wi-Fi
#      80 s  : rechargement du pilote de la carte Wi-Fi
#      puis recommence le cycle toutes les 2 min
#  Journal : /var/log/neo_netguard.log (tourne à 5 Mo)
# =============================================================================
LOG=/var/log/neo_netguard.log
CON="${NEO_WIFI_CON:-Freebox-A39FDE}"
GW="${NEO_GW:-192.168.1.254}"

log() { echo "$(date '+%F %T') $*" >> "$LOG"; }

rotate() {
  if [ -f "$LOG" ] && [ "$(stat -c %s "$LOG")" -gt 5000000 ]; then
    mv -f "$LOG" "$LOG.1"
  fi
}

wifi_dev() { nmcli -t -f DEVICE,TYPE device | awk -F: '$2=="wifi"{print $1; exit}'; }

status_line() {
  local dev="$1" ping_ms="$2"
  local link sig rate freq load temp
  link=$(iw dev "$dev" link 2>/dev/null)
  sig=$(echo "$link" | awk '/signal/{print $2}')
  rate=$(echo "$link" | awk '/tx bitrate/{print $3}')
  freq=$(echo "$link" | awk '/freq/{print $2}')
  load=$(cut -d' ' -f1 /proc/loadavg)
  temp=$(( $(cat /sys/class/thermal/thermal_zone*/temp 2>/dev/null | sort -n | tail -1) / 1000 ))
  echo "signal=${sig:-?}dBm debit=${rate:-?}Mb/s freq=${freq:-?} ping=${ping_ms} charge=${load} temp=${temp}C"
}

fails=0
last_status=0
DEV=$(wifi_dev)
log "=== démarrage du gardien (interface ${DEV:-?}, connexion $CON, box $GW) ==="
# Économie d'énergie Wi-Fi : première cause de décrochages sur Jetson
[ -n "$DEV" ] && iw dev "$DEV" set power_save off 2>/dev/null

while true; do
  rotate
  DEV=$(wifi_dev)
  out=$(ping -c 1 -W 2 -I "${DEV:-wlan0}" "$GW" 2>/dev/null)
  if [ $? -eq 0 ]; then
    ms=$(echo "$out" | sed -n 's/.*time=\([0-9.]*\).*/\1/p')
    if [ $fails -ge 3 ]; then
      log "RÉTABLI après $((fails * 5)) s de coupure — $(status_line "$DEV" "${ms}ms")"
    fi
    fails=0
    now=$(date +%s)
    if [ $((now - last_status)) -ge 30 ]; then
      log "ok $(status_line "$DEV" "${ms}ms")"
      last_status=$now
    fi
  else
    fails=$((fails + 1))
    [ $fails -eq 1 ] && log "PERTE — $(status_line "${DEV:-wlan0}" "échec") — ip: $(ip -4 -br addr show "${DEV:-wlan0}" | awk '{print $3}')"
    case $(( fails % 24 )) in
      3)  log "réparation 1 : relance de la connexion $CON"
          nmcli connection up "$CON" >> "$LOG" 2>&1 ;;
      8)  log "réparation 2 : radio Wi-Fi off/on"
          nmcli radio wifi off; sleep 3; nmcli radio wifi on
          sleep 5; nmcli connection up "$CON" >> "$LOG" 2>&1 ;;
      16) DRV=$(basename "$(readlink /sys/class/net/${DEV:-wlan0}/device/driver/module 2>/dev/null)")
          if [ -n "$DRV" ]; then
            log "réparation 3 : rechargement du pilote $DRV"
            modprobe -r "$DRV" >> "$LOG" 2>&1; sleep 3; modprobe "$DRV" >> "$LOG" 2>&1
            sleep 10; DEV=$(wifi_dev)
            [ -n "$DEV" ] && iw dev "$DEV" set power_save off 2>/dev/null
            nmcli connection up "$CON" >> "$LOG" 2>&1
          else
            log "réparation 3 : pilote introuvable"
          fi ;;
    esac
  fi
  sleep 5
done
