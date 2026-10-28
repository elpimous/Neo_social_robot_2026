#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
diagnose_base_lidar.py — Cohérence odométrie / lidar de Néo
===========================================================
Auteur : Vincent FOUCAULT

Fait bouger Néo doucement (≈60° sur place, puis ≈50 cm en avant) et compare
ce que disent les roues (/odom) et ce que voit le lidar (/scan) :

  1. ROTATION   : le décor tourne-t-il dans le bon sens et du bon angle ?
                  -> scan en miroir ? odométrie inversée / mal échelonnée ?
  2. SEUIL      : vitesse linéaire minimale qui fait réellement avancer la base
  3. AVANCE     : direction de l'avant du robot dans le repère du lidar
                  -> vrai angle de montage du lidar (laser_yaw)
                  -> échelle de l'odométrie en translation

PRÉREQUIS : qbo_startup actif, lidar qui tourne, tête verrouillée (le launch
de test peut tourner, mais AUCUN objectif Nav2 en cours). Dégage ~1.2 m
DEVANT Néo et 0.5 m autour. Ctrl+C = arrêt immédiat.

  ros2 run qbo_nav_waypoints diagnose_base_lidar
"""

import math
import threading
import time

import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan


def yaw_of(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


class Diag(Node):
    def __init__(self):
        super().__init__('diagnose_base_lidar')
        self.declare_parameter('cmd_vel_topic', '/qbo_arduqbo/base_ctrl/cmd_vel')
        self.declare_parameter('odom_topic', '/qbo_arduqbo/base_ctrl/odom')
        self.declare_parameter('scan_topic', '/scan')
        self.declare_parameter('configured_laser_yaw_deg', 180.0)   # = arg laser_yaw du launch
        g = lambda n: self.get_parameter(n).value  # noqa: E731
        self.cfg_yaw = float(g('configured_laser_yaw_deg'))
        self.pub = self.create_publisher(Twist, g('cmd_vel_topic'), 10)
        self.create_subscription(Odometry, g('odom_topic'), self._odom, 20)
        self.create_subscription(LaserScan, g('scan_topic'), self._scan, qos_profile_sensor_data)
        self.odom = None
        self.scan = None
        self.scan_n = 0
        self.lock = threading.Lock()

    def _odom(self, m):
        p = m.pose.pose
        with self.lock:
            self.odom = (p.position.x, p.position.y, yaw_of(p.orientation))

    def _scan(self, m):
        with self.lock:
            self.scan = m
            self.scan_n += 1

    # ------------------------------------------------------------------ outils
    def drive(self, vx, wz, duration, stop_cond=None):
        t0 = time.monotonic()
        tw = Twist()
        tw.linear.x = vx
        tw.angular.z = wz
        while time.monotonic() - t0 < duration:
            if stop_cond and stop_cond():
                break
            self.pub.publish(tw)
            time.sleep(0.05)
        self.stop()

    def stop(self):
        for _ in range(10):
            self.pub.publish(Twist())
            time.sleep(0.03)

    def avg_scan(self, n=5):
        """Moyenne (médiane) de n scans consécutifs, robot immobile."""
        time.sleep(0.8)
        start = self.scan_n
        scans = []
        t0 = time.monotonic()
        while len(scans) < n and time.monotonic() - t0 < 5.0:
            if self.scan_n > start + len(scans):
                with self.lock:
                    scans.append(self.scan)
            time.sleep(0.02)
        if not scans:
            raise RuntimeError('aucun scan reçu : le lidar tourne-t-il ?')
        m = scans[-1]
        nb = min(len(s.ranges) for s in scans)
        out = []
        for i in range(nb):
            vals = sorted(s.ranges[i] for s in scans
                          if m.range_min < s.ranges[i] < min(m.range_max, 8.0) and math.isfinite(s.ranges[i]))
            out.append(vals[len(vals) // 2] if len(vals) >= max(2, len(scans) // 2) else float('nan'))
        return m.angle_min, m.angle_increment, out

    def pose(self):
        with self.lock:
            return self.odom

    # ------------------------------------------------------------------ tests
    def test_rotation(self):
        print('\n[1/3] ROTATION : Néo pivote d\'environ 60° vers la GAUCHE (sens trigo)...')
        a0, inc, r0 = self.avg_scan()
        _, _, y0 = self.pose()
        target = math.radians(60)
        self.drive(0.0, 0.35, 8.0, lambda: abs(wrap(self.pose()[2] - y0)) >= target)
        _, _, r1 = self.avg_scan()
        dyaw_odom = wrap(self.pose()[2] - y0)
        n = min(len(r0), len(r1))
        # Rotation de +D du robot => décor vu tourner de -D dans le repère laser :
        # r1(alpha) = r0(alpha + D)  =>  r1[i] = r0[i + D/inc]
        best = None
        max_shift = int(math.radians(120) / abs(inc))
        for s in range(-max_shift, max_shift + 1):
            errs = []
            for i in range(0, n, 2):
                j = (i + s) % n
                if math.isfinite(r1[i]) and math.isfinite(r0[j]):
                    errs.append(abs(r1[i] - r0[j]))
            if len(errs) > n // 8:
                errs.sort()
                score = sum(errs[:int(len(errs) * 0.8)]) / int(len(errs) * 0.8)   # moyenne tronquée (robuste)
                if best is None or score < best[0]:
                    best = (score, s)
        dyaw_scan = best[1] * inc
        print(f'      odométrie : {math.degrees(dyaw_odom):+6.1f}°   |   lidar : {math.degrees(dyaw_scan):+6.1f}°'
              f'   (écart résiduel {best[0] * 100:.1f} cm)')
        return dyaw_odom, dyaw_scan

    def test_deadband(self):
        print('\n[2/3] SEUIL : petites vitesses en avant (1.5 s chacune)...')
        moved_at = None
        for v in (0.03, 0.05, 0.07, 0.09, 0.12):
            x0, y0, _ = self.pose()
            self.drive(v, 0.0, 1.5)
            time.sleep(0.4)
            x1, y1, _ = self.pose()
            d = math.hypot(x1 - x0, y1 - y0)
            ok = d > 0.5 * v * 1.5
            print(f'      consigne {v:.2f} m/s -> parcouru {d * 100:5.1f} cm  {"OK" if ok else "(ne bouge pas / trop peu)"}')
            if ok and moved_at is None:
                moved_at = v
        return moved_at

    def test_forward(self):
        print('\n[3/3] AVANCE : Néo avance d\'environ 25 cm tout droit...')
        a0, inc, r0 = self.avg_scan()
        x0, y0, th0 = self.pose()
        self.drive(0.12, 0.0, 4.0, lambda: math.hypot(self.pose()[0] - x0, self.pose()[1] - y0) >= 0.25)
        a0b, _, r1 = self.avg_scan()
        x1, y1, _ = self.pose()
        d_odom = math.hypot(x1 - x0, y1 - y0)
        # Avance de d selon l'axe x robot, d'azimut phi dans le repère laser :
        # dr(alpha) ≈ -d cos(alpha - phi) = A cos(alpha) + B sin(alpha)
        sxx = sxy = syy = sxr = syr = 0.0
        cnt = 0
        for i in range(min(len(r0), len(r1))):
            if math.isfinite(r0[i]) and math.isfinite(r1[i]) and 0.4 < r0[i] < 6.0:
                dr = r1[i] - r0[i]
                if abs(dr) > 0.6:
                    continue
                al = a0 + i * inc
                c, s = math.cos(al), math.sin(al)
                sxx += c * c; sxy += c * s; syy += s * s; sxr += c * dr; syr += s * dr
                cnt += 1
        det = sxx * syy - sxy * sxy
        if cnt < 30 or abs(det) < 1e-6:
            print('      pas assez de points exploitables (dégage la zone, évite les miroirs/vitres)')
            return d_odom, None, None
        A = (sxr * syy - syr * sxy) / det
        B = (syr * sxx - sxr * sxy) / det
        d_scan = math.hypot(A, B)
        phi = math.atan2(-B, -A)          # direction du DÉPLACEMENT dans le repère laser (pose de départ)
        # Direction réelle du déplacement dans le repère robot de départ (odométrie,
        # cap fusionné gyro) : corrige une trajectoire pas parfaitement droite
        delta = wrap(math.atan2(y1 - y0, x1 - x0) - th0)
        laser_yaw = wrap(delta - phi)     # yaw du lidar par rapport à la base
        print(f'      dérive de cap pendant l\'avance : {math.degrees(delta):+.1f}° (corrigée)')
        print(f'      odométrie : {d_odom * 100:5.1f} cm   |   lidar : {d_scan * 100:5.1f} cm')
        print(f'      avant du robot vu par le lidar à {math.degrees(phi):+.0f}°  =>  laser_yaw mesuré ≈ {math.degrees(laser_yaw):+.0f}°')
        return d_odom, d_scan, laser_yaw


def main(args=None):
    rclpy.init(args=args)
    node = Diag()
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()
    print(__doc__.split('\n\n')[1])
    try:
        t0 = time.monotonic()
        while (node.pose() is None or node.scan is None) and time.monotonic() - t0 < 10:
            time.sleep(0.1)
        if node.pose() is None:
            raise RuntimeError('pas d\'odométrie reçue')
        if node.scan is None:
            raise RuntimeError('pas de /scan reçu')
        input('\n>>> 1.2 m dégagés DEVANT Néo, aucun objectif Nav2 en cours ? Entrée pour commencer (Ctrl+C annule) ')

        dyo, dys = node.test_rotation()
        vmin = node.test_deadband()
        d_odom, d_scan, lyaw = node.test_forward()

        print('\n=================== DIAGNOSTIC ===================')
        verdict_ok = True
        if abs(dyo) < math.radians(20):
            print('✗ La base n\'a presque pas tourné selon l\'odométrie : vérifie la commande angulaire.')
            verdict_ok = False
        elif dys * dyo < 0:
            print('✗ ROTATION EN SENS OPPOSÉ : soit le scan est en MIROIR (lidar retourné ->')
            print('  inverted:=true sur rplidar_node), soit l\'odométrie a le signe de rotation inversé.')
            print('  Pour trancher : pivote Néo À LA MAIN vers la gauche, `ros2 topic echo')
            print('  /qbo_arduqbo/base_ctrl/odom --field pose.pose.orientation.z` doit AUGMENTER.')
            verdict_ok = False
        else:
            ratio = dys / dyo
            if abs(ratio - 1.0) > 0.15:
                print(f'✗ ÉCHELLE DE ROTATION FAUSSE : le robot tourne réellement {ratio:.2f}× ce que dit')
                print('  l\'odométrie -> corriger l\'entraxe des roues dans le firmware/qbo_arduqbo')
                print(f'  (entraxe_actuel × {1 / ratio:.2f}).')
                verdict_ok = False
            else:
                print(f'✓ Rotation cohérente (lidar/odom = {ratio:.2f})')
        if vmin is None:
            print('✗ La base n\'avance pas même à 0.12 m/s !')
            verdict_ok = False
        elif vmin > 0.06:
            print(f'✗ SEUIL DE DÉMARRAGE {vmin:.2f} m/s : Nav2 descend à 0.04–0.08 m/s près des meubles')
            print(f'  -> régler regulated_linear_scaling_min_speed / min_approach_linear_velocity ≥ {vmin + 0.02:.2f}')
            verdict_ok = False
        else:
            print(f'✓ La base démarre dès {vmin:.2f} m/s')
        if d_scan is not None:
            r = d_scan / max(d_odom, 1e-3)
            if not 0.65 < r < 1.4:   # estimation lidar ~20 % basse par construction (approximation linéaire)
                print(f'✗ ÉCHELLE DE TRANSLATION : lidar/odom = {r:.2f} (rayon de roue / ticks à corriger)')
                verdict_ok = False
            else:
                print(f'✓ Translation cohérente (lidar/odom = {r:.2f})')
            err = wrap(lyaw - math.radians(node.cfg_yaw))
            if abs(err) > math.radians(12):   # précision de la mesure ≈ ±8°
                print(f'✗ ANGLE DE MONTAGE DU LIDAR : mesuré {math.degrees(lyaw):+.0f}°, configuré {node.cfg_yaw:+.0f}°')
                print(f'  -> laser_yaw = {lyaw:.3f} rad (à me transmettre)')
                verdict_ok = False
            else:
                print(f'✓ Angle de montage du lidar cohérent ({math.degrees(lyaw):+.0f}°)')
        print('==================================================')
        print('Tout est cohérent.' if verdict_ok else 'Copie ce bloc DIAGNOSTIC à Claude.')
    except KeyboardInterrupt:
        print('\nInterrompu.')
    except Exception as e:  # noqa: BLE001
        print(f'\nErreur : {e}')
    finally:
        node.stop()
        executor.shutdown()            # arrêt propre du thread de spin (évite « terminate called »)
        spin_thread.join(timeout=2.0)
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
