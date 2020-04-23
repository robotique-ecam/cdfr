#!/usr/bin/env python3


"""Generate Webots world for simulation."""


from magic_points import GREEN_CUPS, RED_CUPS

z = 0
reversed_cup = 0.132

header = """#VRML_SIM R2020a utf8
WorldInfo {
  basicTimeStep 8
  FPS 30
  defaultDamping Damping {
  }
}
Viewpoint {
  orientation -1 0 0 0.5
  position 1.5 2.5 5.0
}
TexturedBackground {
}
TexturedBackgroundLight {
}
Table {
  rotation_girouette 3.14
}
Asterix {
  translation 0.29 0.17 0.67
  rotation 1 0 0 -1.57
}
"""

file = open('worlds/cdr2020.wbt', 'w')

file.write(header)

for name, red in RED_CUPS.items():
    file.write('RedSignal {\n   name \"' + name + '\"\n   translation ' + str(red[0]) + ' ')
    if int(name[3:]) < 25:
        file.write(str(z) + ' ' + str(round(2 - red[1], 3)) + '\n}\n')
    else:
        file.write(str(z + reversed_cup) + ' ' + str(round(2 - red[1], 3)) + '\n   rotation 1 0 0 3.1415\n}\n')
for name, green in GREEN_CUPS.items():
    file.write('GreenSignal {\n   name \"' + name + '\"\n   translation ' + str(green[0]) + ' ')
    if int(name[3:]) < 25:
        file.write(str(z) + ' ' + str(round(2 - green[1], 3)) + '\n}\n')
    else:
        file.write(str(z + reversed_cup) + ' ' + str(round(2 - green[1], 3)) + '\n   rotation 1 0 0 3.1415\n}\n')
file.close()
