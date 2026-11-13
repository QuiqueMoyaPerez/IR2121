#!/usr/bin/env python3
# Junta todos los map_XX.pgm de esta carpeta en maps.pdf (una página por mapa)
# y, si existe overlay.png (de overlay_td.py), lo añade como última página
import glob
from PIL import Image, ImageDraw, ImageFont

W, TOP = 1000, 60
try:
    font = ImageFont.truetype("/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf", 28)
except OSError:
    font = ImageFont.load_default()

pages = []
for yaml in sorted(glob.glob("map_*.yaml")):
    info = dict(l.split(":", 1) for l in open(yaml) if ":" in l)
    img = Image.open(yaml[:-5] + ".pgm").convert("RGB")
    s = (W - 40) / img.width
    img = img.resize((W - 40, int(img.height * s)), Image.NEAREST)
    page = Image.new("RGB", (W, img.height + TOP + 20), "white")
    page.paste(img, (20, TOP))
    text = f"{yaml[:-5]}   resolution: {info['resolution'].strip()}   origin: {info['origin'].strip()}"
    ImageDraw.Draw(page).text((20, 15), text, fill="black", font=font)
    pages.append(page)

import os
if os.path.exists("overlay.png"):
    ov = Image.open("overlay.png").convert("RGB")
    ov.thumbnail((2000, 2000))
    pages.append(ov)

pages[0].save("maps.pdf", save_all=True, append_images=pages[1:])
print(f"maps.pdf creado con {len(pages)} páginas")
