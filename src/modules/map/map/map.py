#!/usr/bin/env python3

"""Script generating maps for CDR2022."""

import os
from PIL import Image, ImageDraw

black = (0, 0, 0)
white = (255, 255, 255)
background_color = (150, 150, 150)

length_cm = 304
width_cm = 204
offset_cm = 400

reso = 2


def cm_to_pix(taille_cm, cmpar_pixel):
    """Return cm per pixel."""
    return taille_cm // cmpar_pixel


def create_map():
    """Create basemap."""
    map = Image.new(
        "RGB", (cm_to_pix(length_cm, reso), cm_to_pix(width_cm, reso)), white
    )

    draw = ImageDraw.Draw(map)

    # Borders
    draw.line(((0, 0), (0, cm_to_pix(width_cm, reso))), black, 2 // reso)
    draw.line(((0, 0), (cm_to_pix(length_cm, reso), 0)), black, 2 // reso)
    draw.line(
        (
            (0, cm_to_pix(width_cm, reso) - 2 // reso),
            (cm_to_pix(length_cm, reso), cm_to_pix(width_cm, reso) - 2 // reso),
        ),
        black,
        2 // reso,
    )
    draw.line(
        (
            (cm_to_pix(length_cm, reso) - 2 // reso, 0),
            (cm_to_pix(length_cm, reso) - 2 // reso, cm_to_pix(width_cm, reso)),
        ),
        black,
        2 // reso,
    )

    # Abris de chantier
    draw.line(
        (
            (0, cm_to_pix(149 + reso, reso)),
            (cm_to_pix(51 + reso, reso), cm_to_pix(width_cm, reso)),
        ),
        black,
        1,
    )
    draw.line(
        (
            (cm_to_pix(length_cm, reso), cm_to_pix(149 + reso, reso)),
            (cm_to_pix(249 + reso, reso), cm_to_pix(width_cm, reso)),
        ),
        black,
        1,
    )

    # Barre
    draw.line(
        (
            (cm_to_pix(150 + reso, reso), 0),
            (cm_to_pix(150 + reso, reso), cm_to_pix(30 + reso, reso)),
        ),
        black,
        1,
    )

    # Distributeurs
    draw.line(
        (
            (0, cm_to_pix(118 + reso, reso)),
            (cm_to_pix(10.2 + reso, reso), cm_to_pix(118 + reso, reso)),
        ),
        black,
        1,
    )
    draw.line(
        (
            (cm_to_pix(10.2 + reso, reso), cm_to_pix(118 + reso, reso)),
            (cm_to_pix(10.2 + reso, reso), cm_to_pix(133 + reso, reso)),
        ),
        black,
        1,
    )
    draw.line(
        (
            (0, cm_to_pix(133 + reso, reso)),
            (cm_to_pix(10.2 + reso, reso), cm_to_pix(133 + reso, reso)),
        ),
        black,
        1,
    )
    draw.line(
        (
            (cm_to_pix(length_cm, reso), cm_to_pix(118 + reso, reso)),
            (cm_to_pix(length_cm - 10.2 + reso, reso), cm_to_pix(118 + reso, reso)),
        ),
        black,
        1,
    )
    draw.line(
        (
            (cm_to_pix(length_cm - 10.2 + reso, reso), cm_to_pix(118 + reso, reso)),
            (cm_to_pix(length_cm - 10.2 + reso, reso), cm_to_pix(133 + reso, reso)),
        ),
        black,
        1,
    )
    draw.line(
        (
            (cm_to_pix(length_cm, reso), cm_to_pix(133 + reso, reso)),
            (cm_to_pix(length_cm - 10.2 + reso, reso), cm_to_pix(133 + reso, reso)),
        ),
        black,
        1,
    )
    draw.line(
        (
            (cm_to_pix(128 + reso, reso), 0),
            (cm_to_pix(128 + reso, reso), cm_to_pix(10.2 + reso, reso)),
        ),
        black,
        1,
    )
    draw.line(
        (
            (cm_to_pix(128 + reso, reso), cm_to_pix(10.2 + reso, reso)),
            (cm_to_pix(143 + reso, reso), cm_to_pix(10.2 + reso, reso)),
        ),
        black,
        1,
    )
    draw.line(
        (
            (cm_to_pix(143 + reso, reso), 0),
            (cm_to_pix(143 + reso, reso), cm_to_pix(10.2 + reso, reso)),
        ),
        black,
        1,
    )
    draw.line(
        (
            (cm_to_pix(158 + reso, reso), 0),
            (cm_to_pix(158 + reso, reso), cm_to_pix(10.2 + reso, reso)),
        ),
        black,
        1,
    )
    draw.line(
        (
            (cm_to_pix(158 + reso, reso), cm_to_pix(10.2 + reso, reso)),
            (cm_to_pix(173 + reso, reso), cm_to_pix(10.2 + reso, reso)),
        ),
        black,
        1,
    )
    draw.line(
        (
            (cm_to_pix(173 + reso, reso), 0),
            (cm_to_pix(173 + reso, reso), cm_to_pix(10.2 + reso, reso)),
        ),
        black,
        1,
    )

    # Galeries
    draw.line(
        (
            (cm_to_pix(45 + reso, reso), cm_to_pix(8.5 + reso, reso)),
            (cm_to_pix(117 + reso, reso), cm_to_pix(8.5 + reso, reso)),
        ),
        black,
        1,
    )
    draw.line(
        (
            (cm_to_pix(183 + reso, reso), cm_to_pix(8.5 + reso, reso)),
            (cm_to_pix(255 + reso, reso), cm_to_pix(8.5 + reso, reso)),
        ),
        black,
        1,
    )

    return (map, draw)


def add_background(img):
    """Create background for map."""
    back = Image.new(
        "RGB",
        (cm_to_pix(length_cm + offset_cm, reso), cm_to_pix(width_cm + offset_cm, reso)),
        background_color,
    )
    bg_w, bg_h = back.size
    img_w, img_h = img.size
    offset = ((bg_w - img_w) // 2, (bg_h - img_h) // 2)
    back.paste(img, offset)
    return back


def create_blue_map():
    """Add blue side specific elements."""
    mapbleu, draw = create_map()
    # draw.line(((cm_to_pix(240, reso), 0), (cm_to_pix(240, reso),
    #                                     cm_to_pix(width_cm, reso))), black, 1)
    mapbleu = add_background(mapbleu)
    mapbleu.save(os.path.join(os.getcwd(), "mapbleu.pgm"))


def create_yellow_map():
    """Add yellow side specific elements."""
    mapjaune, draw = create_map()
    # draw.line(((cm_to_pix(60, reso), 0), (cm_to_pix(60, reso),
    #                                    cm_to_pix(width_cm, reso))), black, 1)
    mapjaune = add_background(mapjaune)
    mapjaune.save(os.path.join(os.getcwd(), "mapjaune.pgm"))


if __name__ == "__main__":
    create_blue_map()
    create_yellow_map()
