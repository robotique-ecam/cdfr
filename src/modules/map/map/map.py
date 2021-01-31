#!/usr/bin/env python3


"""Script generating maps for CDR2020."""


import sys
from os import path

from PIL import Image, ImageDraw

noir = (0, 0, 0)
blanc = (255, 255, 255)
bg_color = (150, 150, 150)

longueur_cm = 304
largeur_cm = 204
offset_cm = 400

reso = 2


def cm_pix(taille_cm, cmpar_pixel):
    """Return cm per pixel."""
    return taille_cm // cmpar_pixel


def creamap():
    """Create basemap."""
    map = Image.new("RGB", (cm_pix(longueur_cm, reso), cm_pix(largeur_cm, reso)), blanc)

    draw = ImageDraw.Draw(map)

    draw.line(((0, 0), (0, cm_pix(largeur_cm, reso))), noir, 2 // reso)
    draw.line(((0, 0), (cm_pix(longueur_cm, reso), 0)), noir, 2 // reso)
    draw.line(
        (
            (0, cm_pix(largeur_cm, reso) - 2 // reso),
            (cm_pix(longueur_cm, reso), cm_pix(largeur_cm, reso) - 2 // reso),
        ),
        noir,
        2 // reso,
    )
    draw.line(
        (
            (cm_pix(longueur_cm, reso) - 2 // reso, 0),
            (cm_pix(longueur_cm, reso) - 2 // reso, cm_pix(largeur_cm, reso)),
        ),
        noir,
        2 // reso,
    )

    draw.line(
        (
            (cm_pix(90, reso), cm_pix(largeur_cm, reso)),
            (cm_pix(90, reso), cm_pix(185, reso)),
        ),
        noir,
        1,
    )
    draw.line(
        (
            (cm_pix(210, reso), cm_pix(largeur_cm, reso)),
            (cm_pix(210, reso), cm_pix(185, reso)),
        ),
        noir,
        1,
    )
    draw.line(
        (
            (cm_pix(150, reso), cm_pix(largeur_cm, reso)),
            (cm_pix(150, reso), cm_pix(170, reso)),
        ),
        noir,
        1,
    )

    return (map, draw)


def add_background(img):
    """Create background for map."""
    back = Image.new(
        "RGB",
        (cm_pix(longueur_cm + offset_cm, reso), cm_pix(largeur_cm + offset_cm, reso)),
        bg_color,
    )
    bg_w, bg_h = back.size
    img_w, img_h = img.size
    offset = ((bg_w - img_w) // 2, (bg_h - img_h) // 2)
    back.paste(img, offset)
    return back


def creamapbleu():
    """Add blue side specific elements."""
    mapbleu, draw = creamap()
    # draw.line(((cm_pix(240, reso), 0), (cm_pix(240, reso),
    #                                     cm_pix(largeur_cm, reso))), noir, 1)
    mapbleu = add_background(mapbleu)
    mapbleu.save(path.join(sys.argv[1], "map", "mapbleu.pgm"))


def creamapjaune():
    """Add yellow side specific elements."""
    mapjaune, draw = creamap()
    # draw.line(((cm_pix(60, reso), 0), (cm_pix(60, reso),
    #                                    cm_pix(largeur_cm, reso))), noir, 1)
    mapjaune = add_background(mapjaune)
    mapjaune.save(path.join(sys.argv[1], "map", "mapjaune.pgm"))


if __name__ == "__main__":
    creamapbleu()
    creamapjaune()
