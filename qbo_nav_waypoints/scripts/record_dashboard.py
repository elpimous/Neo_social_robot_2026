#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
record_dashboard.py — Enregistre en vidéo (25 i/s) le tableau de bord web de Néo
=================================================================================
Auteur : Vincent FOUCAULT

À lancer sur le PC (pas sur Néo : le rendu de la page ne doit pas fausser le test
d'autonomie). Ouvre http://192.168.1.23:8080 dans un Chromium invisible, à la
taille demandée, et filme la page jusqu'à Ctrl+C (ou --duree).
La page est mise à l'échelle pour tenir entièrement dans l'image (--fit, par défaut).
Si Néo s'éteint, la page reste figée sur ses dernières valeurs et la vidéo continue.

Installation (une fois, sur le PC Ubuntu) :
    python3 -m venv ~/neo_rec && ~/neo_rec/bin/pip install playwright && ~/neo_rec/bin/playwright install --with-deps chromium

Utilisation :
    ~/neo_rec/bin/python record_dashboard.py                        # 1920x1080, jusqu'à Ctrl+C
    ~/neo_rec/bin/python record_dashboard.py --taille 2560x1440 --duree 30
    ~/neo_rec/bin/python record_dashboard.py --url http://192.168.1.23:8080 --sortie ~/Vidéos/neo.mp4
"""

import argparse
import os
import shutil
import signal
import subprocess
import time
from datetime import datetime

from playwright.sync_api import sync_playwright

FIT_JS = """(vh) => {
  // la mise en page change avec le zoom (grille plus large) : on converge en quelques passes
  let z = 1;
  document.body.style.zoom = 1;
  for (let i = 0; i < 8; i++) {
    const h = document.documentElement.scrollHeight;
    if (Math.abs(h - vh) <= 2 && h <= vh) break;
    z = Math.min(1, z * vh / h) * 0.995;
    document.body.style.zoom = z;
  }
  window.dispatchEvent(new Event('resize'));
  return z;
}"""


def main():
    ap = argparse.ArgumentParser(description='Filme le tableau de bord de Néo (25 i/s)')
    ap.add_argument('--url', default='http://192.168.1.23:8080')
    ap.add_argument('--taille', default='1920x1080', help='largeur x hauteur de la vidéo (px)')
    ap.add_argument('--duree', type=float, default=0.0, help='durée en minutes (0 = jusqu\'à Ctrl+C)')
    ap.add_argument('--sortie', default='', help='fichier .mp4 (défaut : ~/Vidéos/neo_dashboard_<date>.mp4)')
    ap.add_argument('--no-fit', action='store_true', help='ne pas réduire la page pour tout faire tenir')
    a = ap.parse_args()

    w, h = (int(v) for v in a.taille.lower().split('x'))
    out = os.path.expanduser(a.sortie or os.path.join(
        '~/Vidéos' if os.path.isdir(os.path.expanduser('~/Vidéos')) else '~',
        datetime.now().strftime('neo_dashboard_%Y%m%d_%H%M%S.mp4')))
    tmp_dir = os.path.join(os.path.dirname(out) or '.', '.neo_rec_tmp')
    os.makedirs(tmp_dir, exist_ok=True)

    stop = {'flag': False}
    signal.signal(signal.SIGINT, lambda *_: stop.update(flag=True))
    signal.signal(signal.SIGTERM, lambda *_: stop.update(flag=True))

    with sync_playwright() as p:
        browser = p.chromium.launch(args=['--force-color-profile=srgb'])
        ctx = browser.new_context(viewport={'width': w, 'height': h}, device_scale_factor=1,
                                  color_scheme='dark', record_video_dir=tmp_dir,
                                  record_video_size={'width': w, 'height': h})
        page = ctx.new_page()
        print(f'Ouverture de {a.url} en {w}x{h}…', flush=True)
        page.goto(a.url, wait_until='domcontentloaded', timeout=20000)
        page.wait_for_timeout(3000)                       # carte + historique chargés
        if not a.no_fit:
            z = page.evaluate(FIT_JS, h)
            print(f'Page mise à l\'échelle : {z * 100:.0f} %', flush=True)
        t0 = time.monotonic()
        print(f'● ENREGISTREMENT (25 i/s)  →  {out}\n  Ctrl+C pour arrêter.', flush=True)
        last = 0
        while not stop['flag']:
            page.wait_for_timeout(500)                    # laisse la page vivre (rafraîchissements)
            el = time.monotonic() - t0
            if a.duree > 0 and el >= a.duree * 60:
                break
            if int(el) // 60 != last:
                last = int(el) // 60
                print(f'  … {last} min', flush=True)
            if not a.no_fit and int(el) % 30 == 0:        # la page grandit (journal) : on réajuste
                page.evaluate(FIT_JS, h)
        print('Arrêt, finalisation de la vidéo…', flush=True)
        video = page.video
        ctx.close()                                       # écrit le .webm
        webm = video.path()
        browser.close()

    if shutil.which('ffmpeg'):
        subprocess.run(['ffmpeg', '-loglevel', 'error', '-y', '-i', webm, '-r', '25', '-c:v', 'libx264',
                        '-preset', 'medium', '-crf', '20', '-pix_fmt', 'yuv420p', '-movflags', '+faststart',
                        out], check=False)
        if os.path.exists(out) and os.path.getsize(out) > 0:
            os.remove(webm)
            print(f'Vidéo : {out}')
        else:
            print(f'Conversion MP4 ratée, vidéo brute : {webm}')
    else:
        dst = os.path.splitext(out)[0] + '.webm'
        shutil.move(webm, dst)
        print(f'Vidéo (WebM, ffmpeg absent pour le MP4) : {dst}')
    shutil.rmtree(tmp_dir, ignore_errors=True)


if __name__ == '__main__':
    main()
