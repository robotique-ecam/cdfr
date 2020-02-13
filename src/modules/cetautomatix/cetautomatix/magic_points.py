#!/usr/bin/env python3

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
    "ARUCO42": (0.75, 1.5, 90),
    "MANCHE1": (0.23, 0, 0),
    "MANCHE2": (0.635, 0, 0),
    "MANCHE3": (2.365, 0, 0),
    "MANCHE4": (2.77, 0, 0),
    "PHARE_BLEU": (0.225, 2, 90),
    "PHARE_JAUNE": (2.775, 2, 90),
    "GOB1": (0.3, 0.8),
    "GOB2": (0.3, 1.6),
    "GOB3": (0.45, 0.92),
    "GOB4": (0.45, 1.49),
    "GOB5": (0.67, 1.9),
    "GOB6": (0.95, 1.6),
    "GOB7": (1.005, 0.045),
    "GOB8": (1.065, 0.35),
    "GOB9": (1.1, 1.2),
    "GOB10": (1.27, 0.8),
    "GOB11": (1.335, 0.35),
    "GOB12": (1.395, 0.045),
    "GOB13": (1.605, 0.045),
    "GOB14": (1.665, 0.35),
    "GOB15": (1.73, 0.8),
    "GOB16": (1.9, 1.2),
    "GOB17": (1.935, 0.35),
    "GOB18": (1.995, 0.045),
    "GOB19": (2.05, 1.6),
    "GOB20": (2.33, 1.9),
    "GOB21": (2.55, 0.92),
    "GOB22": (2.55, 1.49),
    "GOB23": (2.7, 0.8),
    "GOB24": (2.7, 1.6),
    # GOBROUGE: [GOB2, GOB3, GOB5, GOB7, GOB9, GOB11, GOB13, GOB15, GOB17, GOB19, GOB22, GOB23],
    # GOBVERT: [GOB1, GOB4, GOB6, GOB8, GOB10, GOB12, GOB14, GOB16, GOB18, GOB20, GOB21, GOB24],

    # Zones
    "CHENAL_BLEU_ROUGE_1": (0, 0.9, 0.4, 0.93),
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
    "PORT_JAUNE_2": (1.15, 0, 1.25, 0.3)
}
