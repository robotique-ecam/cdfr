#!/usr/bin/env python3


import matplotlib.pyplot as plt

from cetautomatix import magic_points

plt.axes()
border = plt.Rectangle((0, 0), 3, 2, ec="black", fc="w", lw=5)
plt.gca().add_patch(border)
for key, value in magic_points.elements.items():
    if "ECUEIL" in key:
        if "BLEU" in key:
            line = plt.Line2D(
                (value[0], value[0]),
                (value[1] - 0.2095, value[1] + 0.2095),
                lw=5,
                color="b",
            )
            plt.gca().add_line(line)
        elif "JAUNE" in key:
            line = plt.Line2D(
                (value[0], value[0]),
                (value[1] - 0.2095, value[1] + 0.2095),
                lw=5,
                color="y",
            )
            plt.gca().add_line(line)
        else:
            line = plt.Line2D(
                (value[0] - 0.2095, value[0] + 0.2095),
                (value[1], value[1]),
                lw=5,
                color="grey",
            )
            plt.gca().add_line(line)
    elif "ARUCO" in key:
        aruco = plt.Rectangle(
            (value[0] - 0.05, value[1] - 0.05), 0.1, 0.1, ec="black", fc="w", lw=2
        )
        plt.gca().add_patch(aruco)
    elif "MANCHE" in key:
        line = plt.Line2D(
            (value[0] - 0.05, value[0] + 0.05), (value[1], value[1]), lw=5, color="grey"
        )
        plt.gca().add_line(line)
    elif "PHARE" in key:
        if "BLEU" in key:
            line = plt.Line2D(
                (value[0] - 0.225, value[0] + 0.225),
                (value[1], value[1]),
                lw=5,
                color="b",
            )
            plt.gca().add_line(line)
        else:
            line = plt.Line2D(
                (value[0] - 0.225, value[0] + 0.225),
                (value[1], value[1]),
                lw=5,
                color="y",
            )
            plt.gca().add_line(line)
    elif "GOB" in key:
        for key2, value2 in value.items():
            if "VERT" in key:
                circle = plt.Circle(value2, radius=0.03, fc="g")
                plt.gca().add_patch(circle)
            else:
                circle = plt.Circle(value2, radius=0.03, fc="r")
                plt.gca().add_patch(circle)
    elif "ZONES" in key:
        for key2, value2 in value.items():
            if "CHENAL" in key2:
                if "ROUGE" in key2:
                    chenal = plt.Rectangle(
                        (value2[0], value2[1]),
                        value2[2] - value2[0],
                        value2[3] - value2[1],
                        fc="r",
                    )
                    plt.gca().add_patch(chenal)
                else:
                    chenal = plt.Rectangle(
                        (value2[0], value2[1]),
                        value2[2] - value2[0],
                        value2[3] - value2[1],
                        fc="g",
                    )
                    plt.gca().add_patch(chenal)
            elif "PORT" in key2:
                if "BLEU" in key2:
                    port = plt.Rectangle(
                        (value2[0], value2[1]),
                        value2[2] - value2[0],
                        value2[3] - value2[1],
                        ec="b",
                        fc="w",
                    )
                    plt.gca().add_patch(port)
                else:
                    port = plt.Rectangle(
                        (value2[0], value2[1]),
                        value2[2] - value2[0],
                        value2[3] - value2[1],
                        ec="y",
                        fc="w",
                    )
                    plt.gca().add_patch(port)

plt.axis("scaled")
plt.show()
