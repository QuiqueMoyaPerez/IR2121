#!/usr/bin/env python3
"""Superpone el último mapa SLAM (map_XX) sobre el plano del edificio TD (TD_n1).

Uso (desde la carpeta donde están los mapas y TD_n1.png / TD_n1.yaml):
    python3 overlay_td.py                      # busca la alineación automáticamente
    python3 overlay_td.py --hint X Y 10        # busca solo a <=10 m de (X, Y) del plano TD
    python3 overlay_td.py --x X --y Y --yaw G  # alineación manual (pose inicial del robot en TD)
    python3 overlay_td.py --completo           # dibuja el plano TD entero, no solo la zona del mapa
    python3 overlay_td.py --sin-leyenda        # sin el texto de arriba
Genera overlay.png. Solo necesita numpy y PIL.
"""
import argparse
import glob
import math
import sys

import numpy as np
try:                                   # scipy es más rápido (multihilo); si no está, numpy
    from scipy import fft as _fft
    rfft2 = lambda x, s=None: _fft.rfft2(x, s=s, workers=-1)
    irfft2 = lambda x, s=None: _fft.irfft2(x, s=s, workers=-1)
    fastlen = _fft.next_fast_len
except ImportError:
    rfft2, irfft2 = np.fft.rfft2, np.fft.irfft2
    fastlen = lambda n, real=True: int(2 ** np.ceil(np.log2(n)))
from PIL import Image, ImageDraw, ImageFilter, ImageFont


def load_map(yaml_path):
    info = {}
    for line in open(yaml_path):
        if ":" in line:
            k, v = line.split(":", 1)
            info[k.strip()] = v.strip()
    img_name = info["image"].strip("'\"")
    folder = yaml_path.rsplit("/", 1)[0] + "/" if "/" in yaml_path else ""
    a = np.array(Image.open(folder + img_name).convert("L"))
    if int(info.get("negate", "0")):
        a = 255 - a
    res = float(info["resolution"])
    origin = [float(v) for v in info["origin"].strip("[]").split(",")[:2]]
    return a, res, np.array(origin)


def resample(mask, res, r):
    """Máscara booleana (filas con y hacia arriba) -> rejilla de resolución r (fracción ocupada)."""
    h, w = mask.shape
    nw, nh = max(1, round(w * res / r)), max(1, round(h * res / r))
    im = Image.fromarray((mask * 255).astype(np.uint8)).resize((nw, nh), Image.BOX)
    return np.array(im, dtype=np.float32) / 255.0


def raster(P, r, m, shape):
    T = np.zeros(shape, np.float32)
    idx = np.floor((P - m) / r).astype(int)
    T[idx[:, 1], idx[:, 0]] = 1.0
    return T


def rot(th):
    c, s = math.cos(th), math.sin(th)
    return np.array([[c, -s], [s, c]])


def search(td_occ, td_org, r, S_occ, S_free, angles, lam=0.5, window=None, blur=1.0, center=None):
    """Correlación por FFT para cada ángulo. Devuelve lista (score, th, t)."""
    pm = int(10 / r)                             # margen de 10 m: el mapa SLAM puede salirse del plano
    td_occ = np.pad(td_occ, pm)
    td_org = td_org - pm * r
    F = np.array(Image.fromarray((td_occ * 255).astype(np.uint8))
                 .filter(ImageFilter.GaussianBlur(blur)), np.float32) / 255.0
    F = np.clip(F * 2.5, 0, 1)
    G = (td_occ > 0.3).astype(np.float32)
    org = td_org.copy()
    if window is not None:                       # recorte alrededor de una pista
        (x0, y0), (x1, y1) = window
        c0 = np.clip(np.floor((np.array([x0, y0]) - org) / r).astype(int), 0, None)
        c1 = np.clip(np.ceil((np.array([x1, y1]) - org) / r).astype(int), None,
                     [F.shape[1], F.shape[0]])
        F, G = F[c0[1]:c1[1], c0[0]:c1[0]], G[c0[1]:c1[1], c0[0]:c1[0]]
        org = org + c0 * r
    FH, FW = F.shape
    allP = np.vstack([S_occ, S_free])
    D = int(np.ceil(np.hypot(*np.ptp(allP, 0)) / r)) + 4
    shape = (fastlen(FH + D, real=True), fastlen(FW + D, real=True))
    FF = rfft2(F, s=shape)
    FG = rfft2(G, s=shape)
    out = []
    for th in angles:
        R = rot(th)
        Po, Pf = S_occ @ R.T, S_free @ R.T
        m = np.vstack([Po, Pf]).min(0)
        To, Tf = raster(Po, r, m, shape), raster(Pf, r, m, shape)
        th_h, th_w = (np.floor((np.vstack([Po, Pf]).max(0) - m) / r).astype(int) + 1)[::-1]
        if th_h > FH or th_w > FW:
            continue
        no, nf = max(To.sum(), 1), max(Tf.sum(), 1)
        c = irfft2(FF * np.conj(rfft2(To)) / no - lam * FG * np.conj(rfft2(Tf)) / nf, s=shape)
        c = c[:FH - th_h + 1, :FW - th_w + 1]
        if center is not None:           # solo traslaciones con el inicio dentro de la pista
            kx = org[0] + np.arange(c.shape[1]) * r - m[0] - center[0]
            ky = org[1] + np.arange(c.shape[0]) * r - m[1] - center[1]
            c = np.where(kx[None, :] ** 2 + ky[:, None] ** 2 <= center[2] ** 2, c, -1e9)
        k = np.unravel_index(np.argmax(c), c.shape)
        if c[k] <= -1e8:
            continue
        t = org + np.array([k[1], k[0]]) * r - m
        out.append((float(c[k]), th, t))
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("slam", nargs="?", help="map_XX.yaml (por defecto, el último)")
    ap.add_argument("--td", default="TD_n1.yaml")
    ap.add_argument("--x", type=float)
    ap.add_argument("--y", type=float)
    ap.add_argument("--yaw", type=float, help="grados")
    ap.add_argument("--hint", type=float, nargs=3, metavar=("X", "Y", "RADIO"))
    ap.add_argument("--completo", action="store_true", help="dibujar el plano TD entero")
    ap.add_argument("--sin-leyenda", action="store_true", help="no poner el texto de arriba")
    ap.add_argument("-o", "--out", default="overlay.png")
    a = ap.parse_args()

    slam_yaml = a.slam or sorted(glob.glob("map_*.yaml"))[-1]
    sa, sres, sorg = load_map(slam_yaml)
    ta, tres, torg = load_map(a.td)
    sa_up, ta_up = sa[::-1], ta[::-1]                     # fila 0 = y mínima

    occ_i = np.argwhere(sa_up < 50)
    free_i = np.argwhere(sa_up > 250)
    to_w = lambda ij: sorg + (ij[:, ::-1] + 0.5) * sres
    S_occ, S_free_all = to_w(occ_i), to_w(free_i)
    S_free = S_free_all[::max(1, len(S_free_all) // 20000)]
    print(f"{slam_yaml}: {sa.shape[1]}x{sa.shape[0]} px, {len(S_occ)} celdas ocupadas")

    td_mask = ta_up < 128
    if a.yaw is not None and a.x is not None and a.y is not None:
        best = (None, math.radians(a.yaw), np.array([a.x, a.y]))
    else:
        window = None
        if a.hint:
            hx, hy, hr = a.hint
            ext = np.ptp(np.vstack([S_occ, S_free]), 0).max()
            window = ((hx - hr - ext, hy - hr - ext), (hx + hr + ext, hy + hr + ext))
        # 1) búsqueda gruesa: 0.2 m, cada 2 grados
        r = 0.2
        occ_r = resample(td_mask, tres, r)
        coarse = search(occ_r, torg, r, S_occ, S_free,
                        np.radians(np.arange(0, 360, 2)), window=window, center=a.hint)
        coarse.sort(key=lambda c: -c[0])
        cands, used = [], []
        for c in coarse:
            if all(abs((math.degrees(c[1] - u) + 180) % 360 - 180) > 6 for u in used):
                cands.append(c); used.append(c[1])
            if len(cands) == 3:
                break
        # 2) refinado: 0.05 m, +-2 grados cada 0.5, alrededor de cada candidato
        r = 0.05
        occ_r = resample(td_mask, tres, r)
        best = (-1e9, 0, None)
        for sc, th0, t0 in cands:
            P = np.vstack([S_occ, S_free]) @ rot(th0).T + t0
            win = (P.min(0) - 3, P.max(0) + 3)
            ref = search(occ_r, torg, r, S_occ, S_free,
                         th0 + np.radians(np.arange(-2, 2.01, 0.5)), window=win, blur=2.0)
            for c in ref:
                if c[0] > best[0]:
                    best = c
        print(f"Puntuación: {best[0]:.2f}  (>0.3 suele ser buena; si es baja o se ve mal, usa --hint)")

    _, th, t = best
    yaw = (math.degrees(th) + 180) % 360 - 180
    print(f"Pose inicial del robot en el plano TD: x={t[0]:.2f} y={t[1]:.2f} yaw={yaw:.1f} grados")
    print(f"Para repetirla a mano: python3 overlay_td.py --x {t[0]:.2f} --y {t[1]:.2f} --yaw {yaw:.1f}")

    # Render a la resolución del plano TD, recortado alrededor del mapa SLAM
    R = rot(th)
    P = np.vstack([S_occ, S_free_all]) @ R.T + t
    lo, hi = P.min(0) - 3, P.max(0) + 3
    if a.completo:
        lo = np.minimum(lo, torg)
        hi = np.maximum(hi, torg + np.array([ta.shape[1], ta.shape[0]]) * tres)
    c0 = np.floor((lo - torg) / tres).astype(int)
    c1 = np.ceil((hi - torg) / tres).astype(int)
    bg = np.full((c1[1] - c0[1], c1[0] - c0[0]), 255.0, np.float32)   # blanco fuera del plano
    a0 = np.clip(c0, 0, [ta.shape[1], ta.shape[0]])
    a1 = np.clip(c1, 0, [ta.shape[1], ta.shape[0]])
    if (a1 > a0).all():
        bg[a0[1] - c0[1]:a1[1] - c0[1], a0[0] - c0[0]:a1[0] - c0[0]] = ta_up[a0[1]:a1[1], a0[0]:a1[0]]
    H, W = bg.shape
    jj, ii = np.meshgrid(np.arange(W), np.arange(H))
    wx = torg[0] + (c0[0] + jj + 0.5) * tres
    wy = torg[1] + (c0[1] + ii + 0.5) * tres
    q = np.stack([wx - t[0], wy - t[1]], -1) @ R            # TD -> marco SLAM (R^T)
    sj = np.floor((q[..., 0] - sorg[0]) / sres).astype(int)
    si = np.floor((q[..., 1] - sorg[1]) / sres).astype(int)
    inside = (sj >= 0) & (sj < sa.shape[1]) & (si >= 0) & (si < sa.shape[0])
    v = np.full((H, W), 205, np.uint8)
    v[inside] = sa_up[si[inside], sj[inside]]

    rgb = np.repeat((170 + bg * (85 / 255))[..., None], 3, -1)  # plano TD aclarado
    rgb[bg < 128] = (90, 90, 90)                                 # paredes del plano
    free = v > 250
    rgb[free] = rgb[free] * 0.55 + np.array([120, 190, 255]) * 0.45
    occ = np.array(Image.fromarray(((v < 50) * 255).astype(np.uint8))
                   .filter(ImageFilter.MaxFilter(3))) > 0
    rgb[occ] = (220, 30, 30)
    out = Image.fromarray(rgb[::-1].astype(np.uint8))

    # Marca de la pose inicial y leyenda
    d = ImageDraw.Draw(out)
    sx = (t[0] - torg[0]) / tres - c0[0]
    sy = H - ((t[1] - torg[1]) / tres - c0[1])
    L = 0.8 / tres
    d.ellipse([sx - 10, sy - 10, sx + 10, sy + 10], outline=(0, 140, 0), width=4)
    d.line([sx, sy, sx + L * math.cos(th), sy - L * math.sin(th)], fill=(0, 140, 0), width=5)
    lines = [f"{slam_yaml[:-5]} sobre {a.td[:-5]}",
             "rojo: paredes SLAM   azul: libre SLAM   gris: plano TD",
             f"verde: inicio del robot  x={t[0]:.2f} m  y={t[1]:.2f} m  yaw={yaw:.1f}°"]
    size = max(34, out.width // 50)
    while True:                      # letra que quepa en el ancho de la imagen
        try:
            font = ImageFont.truetype("/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf", size)
        except OSError:
            font = ImageFont.load_default(); break
        if max(font.getlength(l) for l in lines) <= out.width - 30 or size <= 12:
            break
        size -= 2
    lh = int(size * 1.4)
    if a.sin_leyenda:
        lines = []
    bar_h = lh * len(lines) + 20 if lines else 0
    final = Image.new("RGB", (out.width, out.height + bar_h), "white")
    d = ImageDraw.Draw(final)
    for n, txt in enumerate(lines):
        d.text((15, 10 + lh * n), txt, fill="black", font=font)
    final.paste(out, (0, bar_h))
    final.save(a.out)
    print(f"{a.out} creado ({final.width}x{final.height})")


if __name__ == "__main__":
    sys.exit(main())
