#!/usr/bin/env python3

actions = {
    "MANCHE1": {"ONLY_SIDE": "blue"},
    "MANCHE2": {"ONLY_SIDE": "blue"},
    "MANCHE3": {"ONLY_SIDE": "yellow"},
    "MANCHE4": {"ONLY_SIDE": "yellow"},
    "PAVILLON": {"STATUS": "PREEMPTED"},
    "ECUEIL_1": {"ONLY_ROBOT": "obelix", "GOBS": ['GOB35', 'GOB36', 'GOB37', 'GOB38', 'GOB39']},
    "ECUEIL_2": {"ONLY_ROBOT": "obelix", "GOBS": ['GOB40', 'GOB41', 'GOB42', 'GOB43', 'GOB44']},
    "ECUEIL_BLEU": {"ONLY_ROBOT": "obelix", "ONLY_SIDE": "blue", "GOBS": ['GOB25', 'GOB26', 'GOB27', 'GOB28', 'GOB29']},
    "ECUEIL_JAUNE": {"ONLY_ROBOT": "obelix", "ONLY_SIDE": "yellow", "GOBS": ['GOB30', 'GOB31', 'GOB32', 'GOB33', 'GOB34']},
    "PHARE_BLEU": {"ONLY_SIDE": "blue"},
    "PHARE_JAUNE": {"ONLY_SIDE": "yellow"},

    "CHENAL_BLEU_VERT_1": {"ONLY_SIDE": "blue"},
    # "CHENAL_BLEU_VERT_2": {"ONLY_SIDE": "blue"},
    "CHENAL_BLEU_ROUGE_1": {"ONLY_SIDE": "blue"},
    # "CHENAL_BLEU_ROUGE_2": {"ONLY_SIDE": "blue"},

    "CHENAL_JAUNE_VERT_1": {"ONLY_SIDE": "yellow"},
    # "CHENAL_JAUNE_VERT_2": {"ONLY_SIDE": "yellow"},
    "CHENAL_JAUNE_ROUGE_1": {"ONLY_SIDE": "yellow"},
    # "CHENAL_JAUNE_ROUGE_2": {"ONLY_SIDE": "yellow"},

    # Red Cups
    'GOB2': {"COLOR": "RED"},
    'GOB3': {"COLOR": "RED"},
    'GOB5': {"COLOR": "RED"},
    'GOB7': {"COLOR": "RED"},
    'GOB9': {"COLOR": "RED"},
    'GOB11': {"COLOR": "RED"},
    'GOB13': {"COLOR": "RED"},
    'GOB15': {"COLOR": "RED"},
    'GOB17': {"COLOR": "RED"},
    'GOB19': {"COLOR": "RED"},
    'GOB22': {"COLOR": "RED"},
    'GOB23': {"COLOR": "RED"},
    # Cups in ECUEIL_BLEU:
    'GOB25': {"COLOR": "RED", "IN_ECUEIL": True, "ONLY_SIDE": "blue"},
    'GOB27': {"COLOR": "RED", "IN_ECUEIL": True, "ONLY_SIDE": "blue"},
    'GOB29': {"COLOR": "RED", "IN_ECUEIL": True, "ONLY_SIDE": "blue"},
    # Cups in ECUEIL_JAUNE:
    'GOB31': {"COLOR": "RED", "IN_ECUEIL": True, "ONLY_SIDE": "yellow"},
    'GOB33': {"COLOR": "RED", "IN_ECUEIL": True, "ONLY_SIDE": "yellow"},
    # Cups in ECUEIL_1 & ECUEIL_2 following the Scenario 1:
    'GOB36': {"COLOR": "RED", "IN_ECUEIL": True},
    'GOB39': {"COLOR": "RED", "IN_ECUEIL": True},
    'GOB41': {"COLOR": "RED", "IN_ECUEIL": True},
    'GOB42': {"COLOR": "RED", "IN_ECUEIL": True},
    'GOB44': {"COLOR": "RED", "IN_ECUEIL": True},
    # Green Cups
    'GOB1': {"COLOR": "GREEN"},
    'GOB4': {"COLOR": "GREEN"},
    'GOB6': {"COLOR": "GREEN"},
    'GOB8': {"COLOR": "GREEN"},
    'GOB10': {"COLOR": "GREEN"},
    'GOB12': {"COLOR": "GREEN"},
    'GOB14': {"COLOR": "GREEN"},
    'GOB16': {"COLOR": "GREEN"},
    'GOB18': {"COLOR": "GREEN"},
    'GOB20': {"COLOR": "GREEN"},
    'GOB21': {"COLOR": "GREEN"},
    'GOB24': {"COLOR": "GREEN"},
    # Cups in ECUEIL_BLEU:
    'GOB26': {"COLOR": "GREEN", "IN_ECUEIL": True, "ONLY_SIDE": "blue"},
    'GOB28': {"COLOR": "GREEN", "IN_ECUEIL": True, "ONLY_SIDE": "blue"},
    # Cups in ECUEIL_JAUNE:
    'GOB30': {"COLOR": "GREEN", "IN_ECUEIL": True, "ONLY_SIDE": "yellow"},
    'GOB32': {"COLOR": "GREEN", "IN_ECUEIL": True, "ONLY_SIDE": "yellow"},
    'GOB34': {"COLOR": "GREEN", "IN_ECUEIL": True, "ONLY_SIDE": "yellow"},
    # Cups in ECUEIL_1 & ECUEIL_2 following the Scenario 1:
    'GOB35': {"COLOR": "GREEN", "IN_ECUEIL": True},
    'GOB37': {"COLOR": "GREEN", "IN_ECUEIL": True},
    'GOB38': {"COLOR": "GREEN", "IN_ECUEIL": True},
    'GOB40': {"COLOR": "GREEN", "IN_ECUEIL": True},
    'GOB43': {"COLOR": "GREEN", "IN_ECUEIL": True},
}
