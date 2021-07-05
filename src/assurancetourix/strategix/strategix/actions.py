#!/usr/bin/env python3
from strategix.action_objects import Action, Phare, Ecueil, MancheAir, Gobelet

actions = {
    "PAVILLON": Action(tags={"STATUS": "PREEMPT"}),
    # "PHARE_BLEU": Phare(position=(0.26, 2), tags={"ONLY_SIDE": "blue"}),
    # "PHARE_JAUNE": Phare(position=(2.775, 2), tags={"ONLY_SIDE": "yellow"}),
    # "MANCHE1": MancheAir(position=(0.23, 0), tags={"ONLY_SIDE": "blue"}),
    # "MANCHE2": MancheAir(position=(0.635, 0), tags={"ONLY_SIDE": "blue"}),
    # "MANCHE3": MancheAir(position=(2.365, 0), tags={"ONLY_SIDE": "yellow"}),
    # "MANCHE4": MancheAir(position=(2.77, 0), tags={"ONLY_SIDE": "yellow"}),
    # "ECUEIL_1": Ecueil(
    #     position=(0.85, 2),
    #     rotation=90,
    #     gob_list=["GOB35", "GOB36", "GOB37", "GOB38", "GOB39"],
    #     tags={"ONLY_ROBOT": "obelix"},
    # ),
    # "ECUEIL_2": Ecueil(
    #     position=(2.15, 2),
    #     rotation=90,
    #     gob_list=["GOB40", "GOB41", "GOB42", "GOB43", "GOB44"],
    #     tags={"ONLY_ROBOT": "obelix"},
    # ),
    # "ECUEIL_BLEU": Ecueil(
    #     position=(0, 0.4),
    #     rotation=180,
    #     gob_list=["GOB25", "GOB26", "GOB27", "GOB28", "GOB29"],
    #     tags={"ONLY_ROBOT": "obelix", "ONLY_SIDE": "blue"},
    # ),
    # "ECUEIL_JAUNE": Ecueil(
    #     position=(3, 0.4),
    #     rotation=0,
    #     gob_list=["GOB30", "GOB31", "GOB32", "GOB33", "GOB34"],
    #     tags={"ONLY_ROBOT": "obelix", "ONLY_SIDE": "yellow"},
    # ),
    # "GOB1": Gobelet(position=(0.3, 0.8), color="GREEN"),
    # "GOB2": Gobelet(position=(0.3, 1.6), color="RED"),
    # "GOB3": Gobelet(position=(0.45, 0.92), color="RED"),
    # "GOB4": Gobelet(position=(0.45, 1.49), color="GREEN"),
    # "GOB5": Gobelet(position=(0.67, 1.9), color="RED"),
    # "GOB6": Gobelet(position=(0.95, 1.6), color="GREEN"),
    # "GOB7": Gobelet(position=(1.005, 0.045), color="RED"),
    # "GOB8": Gobelet(position=(1.065, 0.35), color="GREEN"),
    # "GOB9": Gobelet(position=(1.1, 1.2), color="RED"),
    "GOB10": Gobelet(position=(1.27, 0.8), color="GREEN"),
    # "GOB11": Gobelet(position=(1.335, 0.35), color="RED"),
    # "GOB12": Gobelet(position=(1.395, 0.045), color="GREEN"),
    # "GOB13": Gobelet(position=(1.605, 0.045), color="RED"),
    # "GOB14": Gobelet(position=(1.665, 0.35), color="GREEN"),
    # "GOB15": Gobelet(position=(1.73, 0.8), color="RED"),
    # "GOB16": Gobelet(position=(1.9, 1.2), color="GREEN"),
    # "GOB17": Gobelet(position=(1.935, 0.35), color="RED"),
    # "GOB18": Gobelet(position=(1.995, 0.045), color="GREEN"),
    # "GOB19": Gobelet(position=(2.05, 1.6), color="RED"),
    # "GOB20": Gobelet(position=(2.33, 1.9), color="GREEN"),
    # "GOB21": Gobelet(position=(2.55, 0.92), color="GREEN"),
    # "GOB22": Gobelet(position=(2.55, 1.49), color="RED"),
    # "GOB23": Gobelet(position=(2.7, 0.8), color="RED"),
    # "GOB24": Gobelet(position=(2.7, 1.6), color="GREEN"),
    # "GOB25": Gobelet(
    #     position=(0, 1), color="RED", tags={"IN_ECUEIL": True, "ONLY_SIDE": "blue"}
    # ),
}

# actions = {
#     "CHENAL_BLEU_VERT_1": {"ONLY_SIDE": "blue", "STATUS": "PREEMPTED"},
#     # "CHENAL_BLEU_VERT_2": {"ONLY_SIDE": "blue"},
#     "CHENAL_BLEU_ROUGE_1": {"ONLY_SIDE": "blue", "STATUS": "PREEMPTED"},
#     # "CHENAL_BLEU_ROUGE_2": {"ONLY_SIDE": "blue"},
#     "CHENAL_JAUNE_VERT_1": {"ONLY_SIDE": "yellow", "STATUS": "PREEMPTED"},
#     # "CHENAL_JAUNE_VERT_2": {"ONLY_SIDE": "yellow"},
#     "CHENAL_JAUNE_ROUGE_1": {"ONLY_SIDE": "yellow", "STATUS": "PREEMPTED"},
#     # "CHENAL_JAUNE_ROUGE_2": {"ONLY_SIDE": "yellow"},
#     # Red Cups
#     # Cups in ECUEIL_BLEU:
#     "GOB25": {"COLOR": "RED", "IN_ECUEIL": True, "ONLY_SIDE": "blue"},
#     "GOB27": {"COLOR": "RED", "IN_ECUEIL": True, "ONLY_SIDE": "blue"},
#     "GOB29": {"COLOR": "RED", "IN_ECUEIL": True, "ONLY_SIDE": "blue"},
#     # Cups in ECUEIL_JAUNE:
#     "GOB31": {"COLOR": "RED", "IN_ECUEIL": True, "ONLY_SIDE": "yellow"},
#     "GOB33": {"COLOR": "RED", "IN_ECUEIL": True, "ONLY_SIDE": "yellow"},
#     # Cups in ECUEIL_1 & ECUEIL_2 following the Scenario 1:
#     "GOB36": {"COLOR": "RED", "IN_ECUEIL": True},
#     "GOB39": {"COLOR": "RED", "IN_ECUEIL": True},
#     "GOB41": {"COLOR": "RED", "IN_ECUEIL": True},
#     "GOB42": {"COLOR": "RED", "IN_ECUEIL": True},
#     "GOB44": {"COLOR": "RED", "IN_ECUEIL": True},
#     # Green Cups
#     # Cups in ECUEIL_BLEU:
#     "GOB26": {"COLOR": "GREEN", "IN_ECUEIL": True, "ONLY_SIDE": "blue"},
#     "GOB28": {"COLOR": "GREEN", "IN_ECUEIL": True, "ONLY_SIDE": "blue"},
#     # Cups in ECUEIL_JAUNE:
#     "GOB30": {"COLOR": "GREEN", "IN_ECUEIL": True, "ONLY_SIDE": "yellow"},
#     "GOB32": {"COLOR": "GREEN", "IN_ECUEIL": True, "ONLY_SIDE": "yellow"},
#     "GOB34": {"COLOR": "GREEN", "IN_ECUEIL": True, "ONLY_SIDE": "yellow"},
#     # Cups in ECUEIL_1 & ECUEIL_2 following the Scenario 1:
#     "GOB35": {"COLOR": "GREEN", "IN_ECUEIL": True},
#     "GOB37": {"COLOR": "GREEN", "IN_ECUEIL": True},
#     "GOB38": {"COLOR": "GREEN", "IN_ECUEIL": True},
#     "GOB40": {"COLOR": "GREEN", "IN_ECUEIL": True},
#     "GOB43": {"COLOR": "GREEN", "IN_ECUEIL": True},
# }
