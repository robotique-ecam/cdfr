#!/usr/bin/env python3

from strategix.action_objects import Phare, Ecueil

# Strategy points are set using (time, coefficient) coordinates
strategies = {
    "NORMAL": {
        "ECUEIL_COULEUR": [(0, 0), (25, 0), (50, 50), (100, 50)],
        "ECUEIL_GENERAL": [(0, 50), (100, 50)],
        "PHARE": [(0, 0), (25, 0), (75, 100), (100, 100)],
        "MANCHE": [(0, 0), (100, 75)],
    }
}


def get_time_coeff(time, action, strategy):
    if type(action) is Phare:
        action = "PHARE"
    elif type(action) is Ecueil:
        if action.tags.get("ONLY_SIDE") is None:
            action = "ECUEIL_GENERAL"
        else:
            action = "ECUEIL_COULEUR"
    action_time_coeff_list = strategies.get(strategy).get(action)
    if action_time_coeff_list is None:
        return 0
    for i in range(len(action_time_coeff_list)):
        prev_time_coeff = action_time_coeff_list[i - 1]
        time_coeff = action_time_coeff_list[i]
        if time < time_coeff[0]:
            # Calculate a score value between 0 and 100 depending on the time value using linear extrapolation
            score = prev_time_coeff[1] + (time - prev_time_coeff[0]) * (
                time_coeff[1] - prev_time_coeff[1]
            ) / (time_coeff[0] - prev_time_coeff[0])
            return score
    return action_time_coeff_list[i][-1]
