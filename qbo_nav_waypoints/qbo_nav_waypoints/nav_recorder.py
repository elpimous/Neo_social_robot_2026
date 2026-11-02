#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
nav_recorder.py — enregistreur de vol de la navigation de Néo (+ rapport des blocages)
=====================================================================================
Auteur : Vincent FOUCAULT

Enregistre 10 fois par seconde TOUT ce qu'il faut pour comprendre un blocage :
  position (AMCL), incertitude de localisation, trajet / destination en cours,
  distance restante et nombre de recoveries (retour Nav2), vitesse à CHAQUE étage :
      Nav2 (cmd_vel_nav)  ->  lissée (cmd_vel_smoothed)  ->  base (après collision_monitor)
      ->  vitesse RÉELLE mesurée (odométrie)
  zone de ralentissement / arrêt du collision_monitor, manœuvre en cours (spin,
  backup, wait), passage étroit, tête (pan), batterie, âge du scan.
Plus un journal d'événements (avertissements et erreurs Nav2, trajets, pauses...).

À l'arrêt (Ctrl+C) : analyse automatique des BLOCAGES (robot en navigation qui
n'avance pas) avec, pour chacun, l'étage où la vitesse se perd :
  - Nav2 ne commande rien                 -> planification / contrôleur
  - commande réduite avant la base        -> collision_monitor / lissage
  - commande envoyée, la base ne bouge pas -> zone morte des moteurs, patinage
et un rapport HTML (carte + trace + blocages + courbes) dans ~/.ros/qbo_nav_test/.

  ros2 run qbo_nav_waypoints nav_recorder                  (lancé par neo_autonomous.launch.py)
  ros2 run qbo_nav_waypoints nav_report  <fichier navrec_*.csv>   (rapport d'un ancien enregistrement)
"""

import csv
import math
import os
import re
import sys
import time
from datetime import datetime

import rclpy
import rclpy.executors
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from rclpy.time import Time

from action_msgs.msg import GoalStatusArray
from diagnostic_msgs.msg import DiagnosticArray
from geometry_msgs.msg import PoseWithCovarianceStamped, Twist
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import Odometry
from rcl_interfaces.msg import Log
from sensor_msgs.msg import JointState, LaserScan
from tf2_ros import Buffer, TransformListener

COLS = ['t_s', 'heure', 'x', 'y', 'yaw_deg', 'loc_std', 'nav', 'dist_rest', 'recov', 'trajet', 'cible', 'etat',
        'v_nav', 'w_nav', 'v_lisse', 'w_lisse', 'v_base', 'w_base', 'v_reel', 'w_reel',
        'ralenti', 'stop', 'manoeuvre', 'etroit', 'pan_deg', 'batt_v', 'scan_age']
NUM = {'t_s', 'x', 'y', 'yaw_deg', 'loc_std', 'nav', 'dist_rest', 'recov', 'trajet', 'v_nav', 'w_nav', 'v_lisse',
       'w_lisse', 'v_base', 'w_base', 'v_reel', 'w_reel', 'ralenti', 'stop', 'etroit', 'pan_deg', 'batt_v',
       'scan_age'}
NAV_NODES = ('controller_server', 'planner_server', 'behavior_server', 'bt_navigator', 'collision_monitor',
             'velocity_smoother', 'amcl', 'neo_autonomous', 'waypoint_mission', 'scan_range_filter')


def yaw_of(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class NavRecorder(Node):

    def __init__(self):
        super().__init__('nav_recorder')
        d = self.declare_parameter
        d('rate_hz', 10.0)
        d('log_dir', '~/.ros/qbo_nav_test')
        d('odom_topic', '/qbo_arduqbo/base_ctrl/odom')
        d('base_cmd_topic', '/qbo_arduqbo/base_ctrl/cmd_vel')
        d('base_frame', 'base_footprint')
        g = lambda n: self.get_parameter(n).value  # noqa: E731
        self.base_frame = g('base_frame')
        self.t0 = time.monotonic()
        self.tw = {k: (0.0, 0.0, 0.0) for k in ('nav', 'lisse', 'base')}   # (v, w, t)
        self.odom = (0.0, 0.0)
        self.loc_std = None
        self.fb = None                       # (dist, recov, t)
        self.nav_active = False
        self.leg, self.target, self.state = 0, '', 'attente'
        self.slow = self.stop = self.narrow = 0
        self.behavior = ''
        self.pan = None
        self.batt = None
        self.scan_t = 0.0
        self.tf = Buffer()
        TransformListener(self.tf, self)

        out = os.path.expanduser(g('log_dir'))
        os.makedirs(out, exist_ok=True)
        stamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        self.csv_path = os.path.join(out, f'navrec_{stamp}.csv')
        self.ev_path = os.path.join(out, f'navrec_{stamp}_evenements.csv')
        self.f = open(self.csv_path, 'w', newline='', encoding='utf-8')
        self.w = csv.writer(self.f)
        self.w.writerow(COLS)
        self.fe = open(self.ev_path, 'w', newline='', encoding='utf-8')
        self.we = csv.writer(self.fe)
        self.we.writerow(['t_s', 'heure', 'niveau', 'noeud', 'x', 'y', 'message'])
        self.n_lines = 0

        for topic, key in (('/cmd_vel_nav', 'nav'), ('/cmd_vel_smoothed', 'lisse'), (g('base_cmd_topic'), 'base')):
            self.create_subscription(Twist, topic, lambda m, k=key: self._tw(k, m), 20)
        self.create_subscription(Odometry, g('odom_topic'), self._odom, 20)
        latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(PoseWithCovarianceStamped, '/amcl_pose', self._amcl, latched)
        self.create_subscription(NavigateToPose.Impl.FeedbackMessage, '/navigate_to_pose/_action/feedback',
                                 self._fb, 10)
        self.create_subscription(GoalStatusArray, '/navigate_to_pose/_action/status', self._status, 10)
        self.create_subscription(Log, '/rosout', self._rosout, QoSProfile(depth=500))
        self.create_subscription(JointState, '/joint_states', self._js, 20)
        self.create_subscription(DiagnosticArray, '/diagnostics', self._diag, 10)
        self.create_subscription(LaserScan, '/scan_filtered', self._scan, qos_profile_sensor_data)
        self.create_timer(1.0 / float(g('rate_hz')), self._sample)
        self.create_timer(5.0, lambda: (self.f.flush(), self.fe.flush()))
        self.get_logger().info(f'Enregistreur de navigation : {self.csv_path}')

    # ------------------------------------------------------------------ callbacks
    def _tw(self, k, m):
        self.tw[k] = (m.linear.x, m.angular.z, time.monotonic())

    def _odom(self, m):
        self.odom = (m.twist.twist.linear.x, m.twist.twist.angular.z)

    def _amcl(self, m):
        c = m.pose.covariance
        self.loc_std = math.sqrt(max(c[0], c[7], 0.0))

    def _fb(self, m):
        self.fb = (m.feedback.distance_remaining, m.feedback.number_of_recoveries, time.monotonic())

    def _status(self, m):
        self.nav_active = any(s.status in (1, 2, 3) for s in m.status_list[-3:])   # accepted/executing/canceling

    def _js(self, m):
        for n, p in zip(m.name, m.position):
            if n == 'head_pan_joint':
                self.pan = p

    def _diag(self, m):
        for st in m.status:
            if 'Battery' in st.name:
                for kv in st.values:
                    if kv.key == 'Voltage':
                        try:
                            self.batt = float(kv.value)
                        except ValueError:
                            pass

    def _scan(self, _m):
        self.scan_t = time.monotonic()

    def _rosout(self, m):
        node, txt = m.name.split('.')[-1], m.msg
        if node not in NAV_NODES and 'local_costmap' not in m.name and 'global_costmap' not in m.name:
            return
        if node == 'collision_monitor':
            if 'slowdown' in txt:
                self.slow, self.stop = 1, 0
            elif 'stop' in txt.lower() and 'normal' not in txt:
                self.stop = 1
            elif 'normal operation' in txt:
                self.slow = self.stop = 0
        elif node == 'behavior_server':
            mb = re.search(r'Running (\w+)', txt)
            if mb:
                self.behavior = mb.group(1)
            elif 'completed' in txt or 'failed' in txt.lower() or 'cancel' in txt.lower():
                self.behavior = ''
        elif node in ('neo_autonomous', 'waypoint_mission'):
            mt = re.search(r'Trajet (\d+) .*: (\S+) -> (\S+)', txt)
            if mt:
                self.leg, self.target, self.state = int(mt.group(1)), mt.group(3), 'trajet'
            elif txt.startswith('Pause'):
                self.state = 'pause'
            elif 'Arrivé' in txt or 'Passé par' in txt:
                self.state = 'arrive'
            elif 'Passage étroit' in txt:
                self.narrow = 1
            elif 'Passage dégagé' in txt:
                self.narrow = 0
        keep = (m.level >= 30 or node in ('neo_autonomous', 'waypoint_mission', 'behavior_server', 'collision_monitor')
                or 'Failed' in txt or 'Reached' in txt or 'preempt' in txt or 'Received a goal' in txt)
        if keep and 'Passing new path' not in txt:
            p = self._pose()
            self.we.writerow([f'{time.monotonic() - self.t0:.2f}', datetime.now().strftime('%H:%M:%S'),
                              {10: 'DEBUG', 20: 'INFO', 30: 'WARN', 40: 'ERROR', 50: 'FATAL'}.get(m.level, m.level),
                              node, f'{p[0]:.2f}' if p else '', f'{p[1]:.2f}' if p else '', txt])

    def _pose(self):
        try:
            tr = self.tf.lookup_transform('map', self.base_frame, Time())
            return tr.transform.translation.x, tr.transform.translation.y, yaw_of(tr.transform.rotation)
        except Exception:
            return None

    # ------------------------------------------------------------------ échantillon
    def _sample(self):
        now = time.monotonic()
        p = self._pose()

        def tw(k):
            v, w, t = self.tw[k]
            return (v, w) if now - t < 0.5 else (0.0, 0.0)
        vn, wn = tw('nav')
        vl, wl = tw('lisse')
        vb, wb = tw('base')
        fb = self.fb if self.fb and now - self.fb[2] < 1.0 else None
        row = [f'{now - self.t0:.2f}', datetime.now().strftime('%H:%M:%S.%f')[:-4],
               f'{p[0]:.3f}' if p else '', f'{p[1]:.3f}' if p else '', f'{math.degrees(p[2]):.1f}' if p else '',
               f'{self.loc_std:.3f}' if self.loc_std is not None else '', int(self.nav_active),
               f'{fb[0]:.2f}' if fb else '', fb[1] if fb else '', self.leg, self.target, self.state,
               f'{vn:.3f}', f'{wn:.3f}', f'{vl:.3f}', f'{wl:.3f}', f'{vb:.3f}', f'{wb:.3f}',
               f'{self.odom[0]:.3f}', f'{self.odom[1]:.3f}', self.slow, self.stop, self.behavior, self.narrow,
               f'{math.degrees(self.pan):.1f}' if self.pan is not None else '',
               f'{self.batt:.2f}' if self.batt is not None else '',
               f'{now - self.scan_t:.2f}' if self.scan_t else '']
        self.w.writerow(row)
        self.n_lines += 1

    def close(self):
        for fh in (self.f, self.fe):
            try:
                fh.flush()
                fh.close()
            except Exception:
                pass


# =============================================================================
#  Analyse des blocages + rapport HTML
# =============================================================================
def load_rows(path):
    rows = []
    with open(path, newline='', encoding='utf-8') as f:
        for r in csv.DictReader(f):
            for k in NUM:
                try:
                    r[k] = float(r[k]) if r.get(k, '') != '' else None
                except ValueError:
                    r[k] = None
            rows.append(r)
    return rows


def mean_abs(rows, k):
    v = [abs(r[k]) for r in rows if r[k] is not None]
    return sum(v) / len(v) if v else 0.0


def find_blockages(rows, window_s=8.0, min_move_m=0.15):
    """Périodes de navigation active où le robot parcourt moins de min_move_m en window_s secondes."""
    nav = [r for r in rows if r['nav'] and r['x'] is not None and r['etat'] != 'pause']
    blocked = []
    j = 0
    for i, r in enumerate(nav):
        while nav[j]['t_s'] < r['t_s'] - window_s:
            j += 1
        if r['t_s'] - nav[j]['t_s'] < window_s * 0.9:
            continue
        seg = nav[j:i + 1]
        if any(seg[k + 1]['t_s'] - seg[k]['t_s'] > 2.0 for k in range(len(seg) - 1)):
            continue                                       # trou dans la navigation (nouveau trajet)
        if math.hypot(r['x'] - nav[j]['x'], r['y'] - nav[j]['y']) < min_move_m:
            blocked.append((nav[j]['t_s'], r['t_s']))
    merged = []
    for a, b in blocked:
        if merged and a <= merged[-1][1] + 1.0:
            merged[-1][1] = max(merged[-1][1], b)
        else:
            merged.append([a, b])
    out = []
    for a, b in merged:
        seg = [r for r in rows if a <= r['t_s'] <= b]
        if not seg:
            continue
        rot = 0.0
        for k in range(1, len(seg)):
            if seg[k]['yaw_deg'] is not None and seg[k - 1]['yaw_deg'] is not None:
                dy = (seg[k]['yaw_deg'] - seg[k - 1]['yaw_deg'] + 180) % 360 - 180
                rot += abs(dy)
        st = {k: mean_abs(seg, k) for k in ('v_nav', 'w_nav', 'v_lisse', 'w_lisse', 'v_base', 'w_base',
                                             'v_reel', 'w_reel')}
        slow = sum(1 for r in seg if r['ralenti']) / len(seg)
        stop = sum(1 for r in seg if r['stop']) / len(seg)
        man = sorted({r['manoeuvre'] for r in seg if r['manoeuvre']})
        cmd_n = st['v_nav'] + 0.3 * st['w_nav']
        cmd_b = st['v_base'] + 0.3 * st['w_base']
        real = st['v_reel'] + 0.3 * st['w_reel']
        if cmd_n < 0.02:
            why = 'Nav2 ne commande presque rien (planification ou contrôleur bloqués)'
        elif cmd_b < 0.5 * cmd_n:
            why = (f'commande réduite avant la base ({cmd_b / cmd_n * 100:.0f} % de la consigne Nav2) : '
                   + ('collision_monitor en zone ralentir/stop' if (slow + stop) > 0.3 else 'lissage de vitesse'))
        elif real < 0.4 * cmd_b:
            why = (f'la base reçoit la commande mais bouge à peine ({real / max(cmd_b, 1e-6) * 100:.0f} %) : '
                   'zone morte des moteurs (rotation lente ?), patinage ou obstacle physique')
        else:
            why = 'le robot bouge mais oscille sur place (va-et-vient / rotations)'
        out.append({'t0': a, 't1': b, 'dur': b - a, 'x': seg[0]['x'], 'y': seg[0]['y'], 'rot': rot,
                    'heure': seg[0]['heure'][:8], 'trajet': seg[0]['trajet'], 'cible': seg[0]['cible'],
                    'slow': slow, 'stop': stop, 'man': ', '.join(man), 'why': why, **st})
    return out


def analyze(csv_path):
    rows = load_rows(csv_path)
    if not rows:
        print('Aucune donnée.')
        return
    blocks = find_blockages(rows)
    dur = rows[-1]['t_s'] - rows[0]['t_s']
    dist = 0.0
    prev = None
    for r in rows:
        if r['x'] is not None:
            if prev and math.hypot(r['x'] - prev[0], r['y'] - prev[1]) < 0.5:
                dist += math.hypot(r['x'] - prev[0], r['y'] - prev[1])
            prev = (r['x'], r['y'])
    nav_t = sum(1 for r in rows if r['nav']) / max(1, len(rows)) * dur
    blk_t = sum(b['dur'] for b in blocks)
    lines = [f'Durée {dur / 60:.1f} min | navigation {nav_t / 60:.1f} min | distance {dist:.0f} m | '
             f'{len(blocks)} blocage(s), {blk_t:.0f} s au total ({blk_t / max(nav_t, 1) * 100:.0f} % du temps de navigation)']
    for i, b in enumerate(blocks, 1):
        lines.append(f'#{i} {b["heure"]} trajet {int(b["trajet"] or 0)} -> {b["cible"]}  {b["dur"]:.0f} s  '
                     f'en ({b["x"]:.2f}, {b["y"]:.2f})  rotation {b["rot"]:.0f}°  '
                     f'w Nav2/base/réel {b["w_nav"]:.2f}/{b["w_base"]:.2f}/{b["w_reel"]:.2f} rad/s  '
                     f'v {b["v_nav"]:.2f}/{b["v_base"]:.2f}/{b["v_reel"]:.2f} m/s  '
                     f'ralenti {b["slow"] * 100:.0f} %  {b["man"]}\n     => {b["why"]}')
    txt = '\n'.join(lines)
    base = os.path.splitext(csv_path)[0]
    with open(base + '_blocages.txt', 'w', encoding='utf-8') as f:
        f.write(txt + '\n')
    html_report(base + '.html', rows, blocks, lines[0])
    print(txt)
    print(f'\nRapport : {base}.html\nBlocages : {base}_blocages.txt')


def html_report(path, rows, blocks, headline):
    try:
        from qbo_nav_waypoints.neo_dashboard import load_map, share
        m = load_map(share('qbo_navigation', 'map', 'map.yaml'))
    except Exception:
        m = None
    pts = [r for r in rows if r['x'] is not None]
    if not pts:
        return
    if m:
        ox, oy, res, W, H = m['origin'][0], m['origin'][1], m['res'], m['w'], m['h']
    else:
        xs, ys = [r['x'] for r in pts], [r['y'] for r in pts]
        ox, oy, res = min(xs) - 1, min(ys) - 1, 0.05
        W, H = int((max(xs) - ox + 1) / res), int((max(ys) - oy + 1) / res)
    S = 3.0
    X = lambda x: (x - ox) / res * S  # noqa: E731
    Y = lambda y: (H - (y - oy) / res) * S  # noqa: E731
    svg = [f'<svg viewBox="0 0 {W * S:.0f} {H * S:.0f}" style="width:100%;max-width:760px;background:#0a0f16;border-radius:8px">']
    if m:
        import base64
        cells = base64.b64decode(m['cells'])
        for yy in range(0, H):
            run = None
            for xx in range(W + 1):
                occ = xx < W and cells[yy * W + xx] == 2
                if occ and run is None:
                    run = xx
                elif not occ and run is not None:
                    svg.append(f'<rect x="{run * S:.0f}" y="{yy * S:.0f}" width="{(xx - run) * S:.0f}" height="{S:.0f}" fill="#33e0c8" opacity=".55"/>')
                    run = None
    seg = []
    for a, b in zip(pts, pts[1:]):
        if math.hypot(a['x'] - b['x'], a['y'] - b['y']) > 0.5:
            continue
        v = abs(b['v_reel'] or 0)
        col = f'hsl({min(130, v / 0.3 * 130):.0f},85%,55%)'
        seg.append(f'<line x1="{X(a["x"]):.1f}" y1="{Y(a["y"]):.1f}" x2="{X(b["x"]):.1f}" y2="{Y(b["y"]):.1f}" stroke="{col}" stroke-width="2.5" stroke-linecap="round"/>')
    svg += seg[::1]
    for i, b in enumerate(blocks, 1):
        rr = 6 + min(20, b['dur'] / 3)
        svg.append(f'<circle cx="{X(b["x"]):.1f}" cy="{Y(b["y"]):.1f}" r="{rr:.0f}" fill="rgba(248,113,113,.25)" stroke="#f87171" stroke-width="2"/>')
        svg.append(f'<text x="{X(b["x"]) + rr + 2:.1f}" y="{Y(b["y"]) + 4:.1f}" fill="#f87171" font-size="13" font-weight="700">{i}</text>')
    svg.append('</svg>')
    # courbes temps : vitesse réelle, vitesse base, batterie
    t0, t1 = rows[0]['t_s'], rows[-1]['t_s']
    CW, CH = 1000, 220
    TX = lambda t: 40 + (t - t0) / max(1e-6, t1 - t0) * (CW - 80)  # noqa: E731
    bv = [r['batt_v'] for r in rows if r['batt_v']]
    bmin, bmax = (min(bv) - 0.1, max(bv) + 0.1) if bv else (11, 14)

    def poly(key, lo, hi, col, absval=True, w=1.2):
        p = []
        step = max(1, len(rows) // 1500)
        for r in rows[::step]:
            v = r[key]
            if v is None:
                continue
            v = abs(v) if absval else v
            p.append(f'{TX(r["t_s"]):.1f},{10 + (hi - v) / (hi - lo) * (CH - 30):.1f}')
        return f'<polyline points="{" ".join(p)}" fill="none" stroke="{col}" stroke-width="{w}"/>'
    ch = [f'<svg viewBox="0 0 {CW} {CH}" style="width:100%;background:#0a0f16;border-radius:8px">']
    for b in blocks:
        ch.append(f'<rect x="{TX(b["t0"]):.1f}" y="10" width="{max(2, TX(b["t1"]) - TX(b["t0"])):.1f}" height="{CH - 30}" fill="rgba(248,113,113,.18)"/>')
    ch += [poly('v_base', 0, 0.35, '#4b5a6d'), poly('v_reel', 0, 0.35, '#4ade80', w=1.5),
           poly('w_reel', 0, 1.2, '#60a5fa'), poly('batt_v', bmin, bmax, '#fbbf24', absval=False, w=2)]
    ch.append('</svg>')
    tr = ''.join(
        f'<tr><td>{i}</td><td>{b["heure"]}</td><td>{int(b["trajet"] or 0)} → {b["cible"]}</td><td>{b["dur"]:.0f} s</td>'
        f'<td>({b["x"]:.2f}, {b["y"]:.2f})</td><td>{b["rot"]:.0f}°</td>'
        f'<td>{b["w_nav"]:.2f} / {b["w_base"]:.2f} / {b["w_reel"]:.2f}</td>'
        f'<td>{b["v_nav"]:.2f} / {b["v_base"]:.2f} / {b["v_reel"]:.2f}</td><td>{b["slow"] * 100:.0f} %</td>'
        f'<td>{b["man"]}</td><td class="w">{b["why"]}</td></tr>' for i, b in enumerate(blocks, 1))
    page = f"""<!doctype html><html lang="fr"><head><meta charset="utf-8"><title>Néo · blocages</title>
<style>body{{background:#07090d;color:#d6e2f0;font:14px system-ui,sans-serif;margin:24px}}
h1{{font:600 18px ui-monospace,monospace;color:#33e0c8;letter-spacing:.1em}}p.h{{font:14px ui-monospace,monospace}}
table{{border-collapse:collapse;width:100%;font:12px ui-monospace,monospace;margin-top:14px}}
td,th{{border-bottom:1px solid #1b2533;padding:5px 7px;text-align:left;vertical-align:top}}th{{color:#6f8399;font-weight:500}}
td.w{{color:#fbbf24}}.l{{color:#6f8399;font-size:12px}}</style></head><body>
<h1>NÉO // ENREGISTREMENT DE NAVIGATION — {datetime.now().strftime('%d/%m/%Y %H:%M')}</h1>
<p class="h">{headline}</p>
<div class="l">Trace colorée par la vitesse réelle (rouge = lent, vert = 0.3 m/s) · cercles rouges = blocages numérotés</div>
{''.join(svg)}
<div class="l" style="margin-top:14px"><span style="color:#4ade80">━</span> vitesse réelle
<span style="color:#4b5a6d">━</span> vitesse commandée à la base <span style="color:#60a5fa">━</span> rotation réelle
<span style="color:#fbbf24">━</span> batterie · bandes rouges = blocages</div>
{''.join(ch)}
<table><tr><th>#</th><th>heure</th><th>trajet</th><th>durée</th><th>position</th><th>rotation</th>
<th>w Nav2 / base / réel</th><th>v Nav2 / base / réel</th><th>ralenti</th><th>manœuvres</th><th>diagnostic</th></tr>{tr}</table>
</body></html>"""
    with open(path, 'w', encoding='utf-8') as f:
        f.write(page)


def report_main():
    """Entrée « nav_report » : rapport d'un enregistrement (le plus récent par défaut)."""
    import glob
    args = [a for a in sys.argv[1:] if not a.startswith('--')]
    path = args[0] if args else max(glob.glob(os.path.expanduser('~/.ros/qbo_nav_test/navrec_*[0-9].csv')),
                                     key=os.path.getmtime, default=None)
    if not path:
        print('Aucun enregistrement navrec_*.csv')
        return
    print(f'Analyse de {path}\n')
    analyze(path)


def main(args=None):
    rclpy.init(args=args)
    node = NavRecorder()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.close()
        path = node.csv_path
        node.destroy_node()
        rclpy.try_shutdown()
        try:
            print('\n=== Analyse des blocages ===', flush=True)
            analyze(path)
        except Exception as e:  # noqa: BLE001
            print(f'Analyse impossible : {e} — relancer : ros2 run qbo_nav_waypoints nav_report {path}')


if __name__ == '__main__':
    main()
