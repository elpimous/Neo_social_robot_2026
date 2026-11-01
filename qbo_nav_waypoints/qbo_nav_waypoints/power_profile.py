#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
power_profile.py — profil de consommation de Néo, étape par étape
=================================================================
Auteur : Vincent FOUCAULT

Outil guidé : à chaque étape tu démarres un élément (qbo_startup, navigation,
lidar, tête, roulage...), tu appuies sur Entrée, et l'outil mesure pendant
`step_s` secondes :
  - la tension batterie (QBoard, /diagnostics « Battery Status », pas 0.1 V)
  - la puissance du module Orin NX (capteurs INA3221 : VDD_IN, CPU/GPU, SOC)
  - la charge CPU
puis affiche l'écart par rapport à l'étape de référence.

Résultats dans ~/.ros/qbo_nav_test/ :
  power_AAAAMMJJ_HHMMSS.csv      toutes les mesures (5 par seconde)
  power_AAAAMMJJ_HHMMSS.html     rapport graphique (à ouvrir dans un navigateur)

  ros2 run qbo_nav_waypoints power_profile
  ros2 run qbo_nav_waypoints power_profile --ros-args -p step_s:=45.0

Limite : la QBoard mesure la tension batterie au dixième de volt et aucun
capteur ne mesure le COURANT batterie ; les capteurs INA3221 ne voient que le
module Orin (pas le hub USB, le lidar, les moteurs, les servos). La chute de
tension batterie d'une étape à l'autre donne une tendance, pas des watts.
"""

import csv
import glob
import os
import statistics
import threading
import time
from datetime import datetime

import rclpy
from rclpy.node import Node
from diagnostic_msgs.msg import DiagnosticArray

STEPS = [
    'Jetson seul (qbo_startup ARRÊTÉ : pas de tension batterie, seulement la conso Orin)',
    'qbo_startup lancé (base, tête, caméra) — robot immobile',
    'Navigation lancée (lidar en rotation) — robot immobile',
    'Tête en balayage (ros2 run qbo_nav_waypoints head_sweep)',
    'Robot en mouvement (mode autonome ou navigation)',
]


def read(p):
    try:
        with open(p) as f:
            return f.read().strip()
    except Exception:
        return None


class PowerProfile(Node):

    def __init__(self):
        super().__init__('power_profile')
        self.step_s = float(self.declare_parameter('step_s', 30.0).value)
        self.rate = float(self.declare_parameter('rate_hz', 5.0).value)
        self.batt = None
        self.batt_t = 0.0
        self.create_subscription(DiagnosticArray, '/diagnostics', self._diag, 10)
        self.rails = []
        for hw in glob.glob('/sys/bus/i2c/drivers/ina3221/*/hwmon/hwmon*'):
            for lab in sorted(glob.glob(hw + '/in*_label')):
                i = lab.split('in')[-1].split('_')[0]
                v, c = f'{hw}/in{i}_input', f'{hw}/curr{i}_input'
                if os.path.exists(v) and os.path.exists(c):
                    self.rails.append((read(lab) or f'in{i}', v, c))
        self.cpu_prev = None

    def _diag(self, m):
        for st in m.status:
            if 'Battery' in st.name:
                for kv in st.values:
                    if kv.key == 'Voltage':
                        try:
                            self.batt, self.batt_t = float(kv.value), time.monotonic()
                        except ValueError:
                            pass

    def sample(self):
        p = {}
        for name, vf, cf in self.rails:
            v, c = read(vf), read(cf)
            if v and c and v.isdigit() and c.isdigit():
                p[name] = int(v) * int(c) / 1e6
        f = [int(x) for x in (read('/proc/stat') or 'cpu 0 0 0 0').splitlines()[0].split()[1:]]
        tot, idle = sum(f), f[3] + (f[4] if len(f) > 4 else 0)
        cpu = None
        if self.cpu_prev and tot > self.cpu_prev[0]:
            cpu = 100.0 * (1 - (idle - self.cpu_prev[1]) / (tot - self.cpu_prev[0]))
        self.cpu_prev = (tot, idle)
        batt = self.batt if time.monotonic() - self.batt_t < 5.0 else None
        return batt, p, cpu


def stat(vals, fn):
    v = [x for x in vals if x is not None]
    return fn(v) if v else None


def fmt(v, d=2, unit=''):
    return '—' if v is None else f'{v:.{d}f}{unit}'


def html_report(path, rows, steps, rails):
    """Rapport autonome : courbes tension batterie + puissance VDD_IN, bandes par étape."""
    if not rows:
        return
    t0, t1 = rows[0][0], rows[-1][0]
    W, H, L, R, T, B = 1000, 420, 60, 60, 20, 40
    X = lambda t: L + (t - t0) / max(1e-6, t1 - t0) * (W - L - R)  # noqa: E731
    bv = [r[2] for r in rows if r[2] is not None]
    pin = [r[3] for r in rows if r[3] is not None]
    bmin, bmax = (min(bv) - 0.1, max(bv) + 0.1) if bv else (12, 14)
    pmax = (max(pin) * 1.15) if pin else 20
    YB = lambda v: T + (bmax - v) / (bmax - bmin) * (H - T - B)  # noqa: E731
    YP = lambda v: T + (pmax - v) / pmax * (H - T - B)  # noqa: E731
    cols = ['#1e293b', '#14313a', '#1f2a44', '#2b2440', '#3a2a2a', '#22332a']
    svg = [f'<svg viewBox="0 0 {W} {H}" width="100%" style="background:#0d121a;border-radius:8px">']
    for i, s in enumerate(steps):
        x0, x1 = X(s['t0']), X(s['t1'])
        svg.append(f'<rect x="{x0:.1f}" y="{T}" width="{max(1, x1 - x0):.1f}" height="{H - T - B}" fill="{cols[i % 6]}"/>')
        svg.append(f'<text x="{x0 + 4:.1f}" y="{T + 14}" fill="#94a3b8" font-size="11">{i + 1}</text>')
    for k in range(5):
        y = T + k * (H - T - B) / 4
        svg.append(f'<line x1="{L}" x2="{W - R}" y1="{y:.1f}" y2="{y:.1f}" stroke="#1b2533"/>')
        svg.append(f'<text x="6" y="{y + 4:.1f}" fill="#fbbf24" font-size="11">{bmax - k * (bmax - bmin) / 4:.2f} V</text>')
        svg.append(f'<text x="{W - R + 6}" y="{y + 4:.1f}" fill="#60a5fa" font-size="11">{pmax - k * pmax / 4:.1f} W</text>')
    pb = ' '.join(f'{X(r[0]):.1f},{YB(r[2]):.1f}' for r in rows if r[2] is not None)
    pp = ' '.join(f'{X(r[0]):.1f},{YP(r[3]):.1f}' for r in rows if r[3] is not None)
    svg.append(f'<polyline points="{pp}" fill="none" stroke="#60a5fa" stroke-width="1.5"/>')
    svg.append(f'<polyline points="{pb}" fill="none" stroke="#fbbf24" stroke-width="2"/>')
    svg.append('</svg>')
    tr = ''.join(
        f'<tr><td>{i + 1}</td><td>{s["label"]}</td><td>{fmt(s["bv"])} V</td><td>{fmt(s["bmin"], 1)} V</td>'
        f'<td>{fmt(s["dv"], 2, " V")}</td><td>{fmt(s["pin"], 1)} W</td><td>{fmt(s["pmax"], 1)} W</td>'
        f'<td>{fmt(s["dp"], 1, " W")}</td><td>{fmt(s["cpu"], 0)} %</td></tr>' for i, s in enumerate(steps))
    page = f"""<!doctype html><html lang="fr"><head><meta charset="utf-8"><title>Néo · consommation</title>
<style>body{{background:#07090d;color:#d6e2f0;font:14px system-ui,sans-serif;margin:24px}}
h1{{font:600 18px ui-monospace,monospace;color:#33e0c8;letter-spacing:.1em}}
table{{border-collapse:collapse;margin-top:16px;width:100%;font:13px ui-monospace,monospace}}
td,th{{border-bottom:1px solid #1b2533;padding:6px 8px;text-align:right;white-space:nowrap}}td:nth-child(2),th:nth-child(2){{text-align:left}}
th{{color:#6f8399;font-weight:500}}.l{{color:#6f8399;font-size:12px;margin-top:8px}}</style></head><body>
<h1>NÉO // PROFIL DE CONSOMMATION — {datetime.now().strftime('%d/%m/%Y %H:%M')}</h1>
{''.join(svg)}
<div class="l"><span style="color:#fbbf24">━</span> tension batterie (QBoard, pas 0.1 V) &nbsp;
<span style="color:#60a5fa">━</span> puissance module Orin (VDD_IN) &nbsp; bandes = étapes</div>
<table><tr><th>#</th><th>étape</th><th>batterie moy.</th><th>batterie min</th><th>Δ tension / réf.</th>
<th>Orin moy.</th><th>Orin max</th><th>Δ Orin / réf.</th><th>CPU</th></tr>{tr}</table>
<p class="l">Rails INA3221 : {', '.join(rails) or 'aucun'}. Le hub USB, le lidar, les moteurs et les servos ne
passent pas par ces capteurs : seule la chute de tension batterie les révèle.</p></body></html>"""
    with open(path, 'w', encoding='utf-8') as f:
        f.write(page)


def main(args=None):
    rclpy.init(args=args)
    node = PowerProfile()
    threading.Thread(target=rclpy.spin, args=(node,), daemon=True).start()
    out_dir = os.path.expanduser('~/.ros/qbo_nav_test')
    os.makedirs(out_dir, exist_ok=True)
    stamp = datetime.now().strftime('%Y%m%d_%H%M%S')
    csv_path = os.path.join(out_dir, f'power_{stamp}.csv')
    html_path = os.path.join(out_dir, f'power_{stamp}.html')
    rail_names = [r[0] for r in node.rails]
    vin = next((n for n in rail_names if 'VDD_IN' in n.upper()), rail_names[0] if rail_names else None)
    rows, steps = [], []
    T0 = time.monotonic()
    print('\n=== Profil de consommation de Néo ===')
    print(f'Chaque étape est mesurée {node.step_s:.0f} s. Rails Orin : {", ".join(rail_names) or "aucun"}')
    print('Démarre l\'élément indiqué, attends qu\'il soit stable, puis Entrée.')
    print('Tape un autre nom avant Entrée pour renommer l\'étape, « q » pour terminer.\n')
    with open(csv_path, 'w', newline='', encoding='utf-8') as f:
        w = csv.writer(f)
        w.writerow(['t_s', 'etape', 'batterie_v', 'orin_vdd_in_w', 'cpu_pct'] + [f'{n}_w' for n in rail_names])
        i = 0
        try:
            while True:
                default = STEPS[i] if i < len(STEPS) else f'Étape {i + 1}'
                ans = input(f'[{i + 1}] {default}\n    Entrée pour mesurer > ').strip()
                if ans.lower() == 'q':
                    break
                label = ans or default
                s = {'label': label, 't0': time.monotonic() - T0}
                bvs, pins, cpus = [], [], []
                t_end = time.monotonic() + node.step_s
                while time.monotonic() < t_end:
                    batt, p, cpu = node.sample()
                    t = time.monotonic() - T0
                    pv = p.get(vin) if vin else None
                    rows.append((t, i + 1, batt, pv, cpu))
                    w.writerow([f'{t:.2f}', label, batt if batt is not None else '', f'{pv:.3f}' if pv else '',
                                f'{cpu:.1f}' if cpu is not None else ''] + [f'{p.get(n, 0):.3f}' for n in rail_names])
                    bvs.append(batt)
                    pins.append(pv)
                    cpus.append(cpu)
                    left = int(t_end - time.monotonic())
                    print(f'\r    mesure… {left:3d} s  batterie {fmt(batt)} V  Orin {fmt(pv, 1)} W  '
                          f'CPU {fmt(cpu, 0)} %   ', end='', flush=True)
                    time.sleep(1.0 / node.rate)
                f.flush()
                s['t1'] = time.monotonic() - T0
                s['bv'] = stat(bvs, statistics.mean)
                s['bmin'] = stat(bvs, min)
                s['pin'] = stat(pins, statistics.mean)
                s['pmax'] = stat(pins, max)
                s['cpu'] = stat(cpus, statistics.mean)
                # référence = première étape où la mesure existe (la tension batterie demande qbo_startup)
                rb = next((x for x in steps + [s] if x['bv'] is not None), None)
                rp = next((x for x in steps + [s] if x['pin'] is not None), None)
                s['dv'] = (s['bv'] - rb['bv']) if (s['bv'] is not None and rb) else None
                s['dp'] = (s['pin'] - rp['pin']) if (s['pin'] is not None and rp) else None
                steps.append(s)
                print(f'\r    ✔ batterie moy. {fmt(s["bv"])} V (min {fmt(s["bmin"], 1)}, Δ {fmt(s["dv"])} V) | '
                      f'Orin moy. {fmt(s["pin"], 1)} W (max {fmt(s["pmax"], 1)}, Δ {fmt(s["dp"], 1)} W) | '
                      f'CPU {fmt(s["cpu"], 0)} %\n')
                i += 1
        except (KeyboardInterrupt, EOFError):
            print()
    html_report(html_path, rows, steps, rail_names)
    print('=== Résumé ===')
    for k, s in enumerate(steps):
        print(f'{k + 1}. {s["label"][:55]:55s} batt {fmt(s["bv"])} V (Δ {fmt(s["dv"])})  '
              f'Orin {fmt(s["pin"], 1)} W (Δ {fmt(s["dp"], 1)})')
    print(f'\nMesures : {csv_path}\nRapport  : {html_path}')
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == '__main__':
    main()
