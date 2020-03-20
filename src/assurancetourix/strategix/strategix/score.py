from strategix.actions import MancheAir, Gobelet, Phare, BonPort, Pavillon
from cetautomatix.magic_points import RED_CUPS, GREEN_CUPS


class Score:
    def __init__(self):
        self.cups = [Gobelet(k, "R") for k in RED_CUPS]
        self.cups += [Gobelet(k, "V") for k in GREEN_CUPS]
        self.mancheAir1 = MancheAir("MANCHE1")
        self.mancheAir2 = MancheAir("MANCHE2")
        self.phare = Phare("PHARE")
        self.bonPortGros = BonPort("BonPortGros")
        self.bonPortPetit = BonPort("BonPortPetit")
        self.pavillon = Pavillon("Pavillon")
        self.todoList = [a.name for a in self.cups] + [self.phare.name, self.mancheAir1.name, self.mancheAir2.name]
        self.wipList = []
        self.doneList = []

    def updateScore(self):
        self.score = 0
        # Manche à Air
        numMancheAir = len([action for action in self.doneList if "MancheAir" in action])
        self.score += 5 if numMancheAir == 1 else 15 if numMancheAir == 2 else 0
        # Gobelets
        gobeletsInBase = [action for action in self.doneList if "Gobelet" in action]
        numGobRouge = len([gob for gob in gobeletsInBase if gob.inChenal and gob.color == 'R'])
        numGobVert = len([gob for gob in gobeletsInBase if gob.inChenal and gob.color == 'V'])
        pair = 2 * numGobRouge if numGobRouge < numGobVert else 2 * numGobVert
        self.score += len(gobeletsInBase) + numGobRouge + numGobVert + pair
        # Phare
        if self.phare.name in self.doneList:
            self.score += 15 if self.phare.state == 2 else 5 if self.phare.state == 1 else 2
        else:
            self.score += 2
        # Bon Port
        if self.bonPortGros.pos == self.bonPortPetit.pos == "Good":
            self.score += 10
        elif self.bonPortGros.pos == self.bonPortPetit.pos == "Wrong":
            self.score += 5
        elif self.bonPortGros.pos == "Good" and self.bonPortPetit.pos == "Out":
            self.score += 5
        elif self.bonPortGros.pos == "Out" and self.bonPortPetit.pos == "Good":
            self.score += 5
        else:
            self.score += 0
        # Pavillon
        self.score += 10 if self.pavillon.raised else 0

    def preempt(self, action):
        self.todoList.remove(action)
        self.wipList.append(action)
        self.updateScore()
        return True

    def release(self, action):
        self.wipList.remove(action)
        self.todoList.append(action)
        self.updateScore()
        return True

    def finish(self, action):
        self.wipList.remove(action)
        self.doneList.append(action)
        self.updateScore()
        return True
