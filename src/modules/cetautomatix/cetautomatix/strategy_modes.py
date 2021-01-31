strategies = {
    "NORMAL": {
        "ECUEIL_COULEUR": [[0, 0], [25, 0], [50, 50], [100, 50]],
        "ECUEIL_GENERAL": [[0, 50], [100, 50]],
        "PHARE": [[0, 0], [25, 0], [75, 100], [100, 100]],
    }
}


def get_time_coeff(time, action, strategy):
    if action == "ECUEIL_BLEU" or action == "ECUEIL_JAUNE":
        action = "ECUEIL_COULEUR"
    elif action == "ECUEIL_1" or action == "ECUEIL_2":
        action = "ECUEIL_GENERAL"
    elif "PHARE" in action:
        action = "PHARE"
    element = strategies.get(strategy).get(action)
    if element is None:
        return 0
    for i in range(len(element)):
        if time < element[i][0]:
            # Calculate a score value between 0 and 100 depending on the time value
            score = element[i - 1][1] + (time - element[i - 1][0]) * (
                element[i][1] - element[i - 1][1]
            ) / (element[i][0] - element[i - 1][0])
            return score
    return element[i][-1]
