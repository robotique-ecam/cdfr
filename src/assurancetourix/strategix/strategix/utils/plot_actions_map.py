#!/usr/bin/env python3

import matplotlib.pyplot as plt
from strategix.action_objects import Ecueil, MancheAir, Phare, Gobelet
from strategix.actions import actions

plt.axes()
plt.xlabel("X")
plt.ylabel("Y")
border = plt.Rectangle((0, 0), 3, 2, ec="black", fc="w", lw=5)
plt.gca().add_patch(border)
for action in actions.values():
    if type(action) is Ecueil:
        if action.tags.get("ONLY_SIDE") == "blue":
            line = plt.Line2D(
                (action.position[0], action.position[0]),
                (action.position[1] - 0.2095, action.position[1] + 0.2095),
                lw=5,
                color="b",
            )
            plt.gca().add_line(line)
        elif action.tags.get("ONLY_SIDE") == "yellow":
            line = plt.Line2D(
                (action.position[0], action.position[0]),
                (action.position[1] - 0.2095, action.position[1] + 0.2095),
                lw=5,
                color="y",
            )
            plt.gca().add_line(line)
        else:
            line = plt.Line2D(
                (action.position[0] - 0.2095, action.position[0] + 0.2095),
                (action.position[1], action.position[1]),
                lw=5,
                color="grey",
            )
            plt.gca().add_line(line)
    elif type(action) is MancheAir:
        line = plt.Line2D(
            (action.position[0] - 0.05, action.position[0] + 0.05),
            (action.position[1], action.position[1]),
            lw=5,
            color="grey",
        )
        plt.gca().add_line(line)
    elif type(action) is Phare:
        if action.tags.get("ONLY_SIDE") == "blue":
            line = plt.Line2D(
                (action.position[0] - 0.225, action.position[0] + 0.225),
                (action.position[1], action.position[1]),
                lw=5,
                color="b",
            )
            plt.gca().add_line(line)
        else:
            line = plt.Line2D(
                (action.position[0] - 0.225, action.position[0] + 0.225),
                (action.position[1], action.position[1]),
                lw=5,
                color="y",
            )
            plt.gca().add_line(line)
    elif type(action) is Gobelet:
        if action.color == "GREEN":
            circle = plt.Circle(action.position, radius=0.03, fc="g")
            plt.gca().add_patch(circle)
        else:
            circle = plt.Circle(action.position, radius=0.03, fc="r")
            plt.gca().add_patch(circle)
    # elif "ZONES" in key:
    #     for key2, value2 in value.items():
    #         if "CHENAL" in key2:
    #             if "ROUGE" in key2:
    #                 chenal = plt.Rectangle(
    #                     (value2[0], value2[1]),
    #                     value2[2] - value2[0],
    #                     value2[3] - value2[1],
    #                     fc="r",
    #                 )
    #                 plt.gca().add_patch(chenal)
    #             else:
    #                 chenal = plt.Rectangle(
    #                     (value2[0], value2[1]),
    #                     value2[2] - value2[0],
    #                     value2[3] - value2[1],
    #                     fc="g",
    #                 )
    #                 plt.gca().add_patch(chenal)
    #         elif "PORT" in key2:
    #             if "BLEU" in key2:
    #                 port = plt.Rectangle(
    #                     (value2[0], value2[1]),
    #                     value2[2] - value2[0],
    #                     value2[3] - value2[1],
    #                     ec="b",
    #                     fc="w",
    #                 )
    #                 plt.gca().add_patch(port)
    #             else:
    #                 port = plt.Rectangle(
    #                     (value2[0], value2[1]),
    #                     value2[2] - value2[0],
    #                     value2[3] - value2[1],
    #                     ec="y",
    #                     fc="w",
    #                 )
    #                 plt.gca().add_patch(port)

plt.axis("scaled")
plt.show()
