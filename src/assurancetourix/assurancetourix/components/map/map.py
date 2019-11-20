from PIL import Image, ImageDraw

noir = (0,0,0)
blanc = (255,255,255)

longueur_cm = 304
largeur_cm = 204

reso = 2

def cm_pix(taille_cm, cmpar_pixel):
    return (taille_cm//cmpar_pixel)

def creamap():
    map = Image.new('RGB',(cm_pix(longueur_cm,reso),cm_pix(largeur_cm,reso)),blanc)

    draw = ImageDraw.Draw(map)

    draw.line(((0,0),(0,cm_pix(largeur_cm,reso))),noir,2//reso)
    draw.line(((0,0),(cm_pix(longueur_cm,reso),0)),noir,2//reso)
    draw.line(((0,cm_pix(largeur_cm,reso)-2//reso),(cm_pix(longueur_cm,reso),cm_pix(largeur_cm,reso)-2//reso)),noir,2//reso)
    draw.line(((cm_pix(longueur_cm,reso)-2//reso,0),(cm_pix(longueur_cm,reso)-2//reso,cm_pix(largeur_cm,reso))),noir,2//reso)

    draw.line(((cm_pix(90,reso),cm_pix(largeur_cm,reso)),(cm_pix(90,reso),cm_pix(185,reso))),noir,1)
    draw.line(((cm_pix(210,reso),cm_pix(largeur_cm,reso)),(cm_pix(210,reso),cm_pix(185,reso))),noir,1)
    draw.line(((cm_pix(150,reso),cm_pix(largeur_cm,reso)),(cm_pix(150,reso),cm_pix(170,reso))),noir,1)

    return (map, draw)


def creamapbleu ():
    mapbleu,draw = creamap()
    draw.line(((cm_pix(240,reso),0),(cm_pix(240,reso),cm_pix(largeur_cm,reso))),noir,1)
    mapbleu.save("mapbleu.ppm","ppm")



def creamapjaune ():
    mapjaune,draw = creamap()
    draw.line(((cm_pix(60,reso),0),(cm_pix(60,reso),cm_pix(largeur_cm,reso))),noir,1)
    mapjaune.save("mapjaune.ppm","ppm")
