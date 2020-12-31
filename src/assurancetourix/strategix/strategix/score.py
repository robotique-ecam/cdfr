from strategix.actions import actions


class Score:
    def __init__(self):
        pass
        # self.bonPortGros = BonPort('BonPortGros')
        # self.bonPortPetit = BonPort('BonPortPetit')

    def get_score(self):
        score = 0
        num_manche_air = 0
        num_red_cups = 0
        num_green_cups = 0
        phare_raised = False
        pavillon_raised = False
        for action in actions:
            if actions[action].get("STATUS") == "FINISHED":
                if "MANCHE" in action:
                    num_manche_air += 1
                if "GOB" in action:
                    if actions[action].get("COLOR") == "RED":
                        num_red_cups += 1
                    else:
                        num_green_cups += 1
                if "PHARE" in action:
                    phare_raised = True
                if "PAVILLON" in action:
                    pavillon_raised = True
        # Bon Port
        # if self.bonPortGros.pos == self.bonPortPetit.pos == 'Good':
        #     self.score += 10
        # elif self.bonPortGros.pos == self.bonPortPetit.pos == 'Wrong':
        #     self.score += 5
        # elif self.bonPortGros.pos == 'Good' and self.bonPortPetit.pos == 'Out':
        #     self.score += 5
        # elif self.bonPortGros.pos == 'Out' and self.bonPortPetit.pos == 'Good':
        #     self.score += 5
        # else:
        #     self.score += 0
        # Pavillon
        score += 15 if num_manche_air == 2 else 5 if num_manche_air == 1 else 0
        pair = 2 * num_red_cups if num_red_cups < num_green_cups else 2 * num_green_cups
        score += 2 * (num_red_cups + num_green_cups) + pair
        score += 15 if phare_raised else 2
        score += 10 if pavillon_raised else 0
        return score

    def preempt(self, action, sender):
        actions[action]["STATUS"] = "PREEMPTED"
        actions[action]["EXECUTER"] = sender
        if "ECUEIL" in action:
            for gob in actions[action]["GOBS"]:
                actions[action]["STATUS"] = "PREEMPTED"
                actions[gob]["EXECUTER"] = sender
        return True

    def release(self, action, sender):
        actions[action]["STATUS"] = "RELEASED"
        actions[action]["EXECUTER"] = None
        if "ECUEIL" in action:
            for gob in actions[action]["GOBS"]:
                actions[action]["STATUS"] = "RELEASED"
                actions[gob]["EXECUTER"] = None
        # Don't add failing action again
        return True

    def finish(self, action, sender):
        actions[action]["STATUS"] = "FINISHED"
        actions[action]["EXECUTER"] = sender
        if "ECUEIL" in action:
            for gob in actions[action]["GOBS"]:
                actions[action]["STATUS"] = "FINISHED"
                actions[gob]["EXECUTER"] = sender
        return True
