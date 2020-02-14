#!/usr/bin/env python3

import matplotlib.pyplot as plt

"""
High level representation of Points of Interest on CDR2020 Table (to be used in Behavior Trees).
Points in (x, y, a) for x & y in meters, a in degrees.
Zones in (x0, y0, x1, y1) for x & y in meters.
"""

elements = {
    # Points:
    "ECUEIL_BLEU": (0, 0.4, 180),
    "ECUEIL_JAUNE": (3, 0.4, 0),
    "ECUEIL_1": (0.85, 2, 90),
    "ECUEIL_2": (2.15, 2, 90),
    "ARUCO42": (1.5, 0.75, 90),
    "MANCHE1": (0.23, 0, 0),
    "MANCHE2": (0.635, 0, 0),
    "MANCHE3": (2.365, 0, 0),
    "MANCHE4": (2.77, 0, 0),
    "PHARE_BLEU": (0.225, 2, 90),
    "PHARE_JAUNE": (2.775, 2, 90),

    "GOBROUGE": {"GOB2": (0.3, 1.6),
                 "GOB3": (0.45, 0.92),
                 "GOB5": (0.67, 1.9),
                 "GOB7": (1.005, 0.045),
                 "GOB9": (1.1, 1.2),
                 "GOB11": (1.335, 0.35),
                 "GOB13": (1.605, 0.045),
                 "GOB15": (1.73, 0.8),
                 "GOB17": (1.935, 0.35),
                 "GOB19": (2.05, 1.6),
                 "GOB22": (2.55, 1.49),
                 "GOB23": (2.7, 0.8)},

    "GOBVERT": {"GOB1": (0.3, 0.8),
                "GOB4": (0.45, 1.49),
                "GOB6": (0.95, 1.6),
                "GOB8": (1.065, 0.35),
                "GOB10": (1.27, 0.8),
                "GOB12": (1.395, 0.045),
                "GOB14": (1.665, 0.35),
                "GOB16": (1.9, 1.2),
                "GOB18": (1.995, 0.045),
                "GOB20": (2.33, 1.9),
                "GOB21": (2.55, 0.92),
                "GOB24": (2.7, 1.6)},

    "ZONES": {"CHENAL_BLEU_ROUGE_1": (0, 0.9, 0.4, 0.93),
              "CHENAL_BLEU_VERT_1": (0, 1.47, 0.4, 1.5),
              "CHENAL_BLEU_ROUGE_2": (1.25, 0, 1.35, 0.3),
              "CHENAL_BLEU_VERT_2": (1.05, 0, 1.15, 0.3),
              "CHENAL_JAUNE_ROUGE_1": (2.6, 1.47, 3, 1.5),
              "CHENAL_JAUNE_VERT_1": (2.6, 0.9, 3, 0.93),
              "CHENAL_JAUNE_ROUGE_2": (1.85, 0, 1.95, 0.3),
              "CHENAL_JAUNE_VERT_2": (1.65, 0, 1.75, 0.3),
              "PORT_BLEU_1": (0, 0.93, 0.4, 1.47),
              "PORT_BLEU_2": (1.75, 0, 1.85, 0.3),
              "PORT_JAUNE_1": (2.6, 0.93, 3, 1.47),
              "PORT_JAUNE_2": (1.15, 0, 1.25, 0.3)}
}

plt.axes()
border = plt.Rectangle((0, 0), 3, 2, ec='black', fc='w', lw=5)
plt.gca().add_patch(border)
for key, value in elements.items():
    if "ECUEIL" in key:
        if "BLEU" in key:
            line = plt.Line2D((value[0], value[0]), (value[1] - 0.2095, value[1] + 0.2095), lw=5, color='b')
            plt.gca().add_line(line)
        elif "JAUNE" in key:
            line = plt.Line2D((value[0], value[0]), (value[1] - 0.2095, value[1] + 0.2095), lw=5, color='y')
            plt.gca().add_line(line)
        else:
            line = plt.Line2D((value[0] - 0.2095, value[0] + 0.2095), (value[1], value[1]), lw=5, color='grey')
            plt.gca().add_line(line)
    elif "ARUCO" in key:
        aruco = plt.Rectangle((value[0] - 0.05, value[1] - 0.05), 0.1, 0.1, ec='black', fc='w', lw=2)
        plt.gca().add_patch(aruco)
    elif "MANCHE" in key:
        line = plt.Line2D((value[0] - 0.05, value[0] + 0.05), (value[1], value[1]), lw=5, color='grey')
        plt.gca().add_line(line)
    elif "PHARE" in key:
        if "BLEU" in key:
            line = plt.Line2D((value[0] - 0.225, value[0] + 0.225), (value[1], value[1]), lw=5, color='b')
            plt.gca().add_line(line)
        else:
            line = plt.Line2D((value[0] - 0.225, value[0] + 0.225), (value[1], value[1]), lw=5, color='y')
            plt.gca().add_line(line)
    elif "GOB" in key:
        for key2, value2 in value.items():
            if "VERT" in key:
                circle = plt.Circle(value2, radius=0.03, fc='g')
                plt.gca().add_patch(circle)
            else:
                circle = plt.Circle(value2, radius=0.03, fc='r')
                plt.gca().add_patch(circle)
    elif "ZONES" in key:
        for key2, value2 in value.items():
            if "CHENAL" in key2:
                if "ROUGE" in key2:
                    chenal = plt.Rectangle((value2[0], value2[1]), value2[2] - value2[0], value2[3] - value2[1], fc='r')
                    plt.gca().add_patch(chenal)
                else:
                    chenal = plt.Rectangle((value2[0], value2[1]), value2[2] - value2[0], value2[3] - value2[1], fc='g')
                    plt.gca().add_patch(chenal)
            elif "PORT" in key2:
                if "BLEU" in key2:
                    port = plt.Rectangle((value2[0], value2[1]), value2[2] - value2[0], value2[3] - value2[1], ec='b', fc='w')
                    plt.gca().add_patch(port)
                else:
                    port = plt.Rectangle((value2[0], value2[1]), value2[2] - value2[0], value2[3] - value2[1], ec='y', fc='w')
                    plt.gca().add_patch(port)

plt.axis('scaled')
plt.show()
