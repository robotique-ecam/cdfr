from actions import MancheAir, Gobelet, Phare, BonPort, Pavillon


class Score:
    def __init__(self):
        self.mancheAir1 = MancheAir("MancheAir1")
        self.mancheAir2 = MancheAir("MancheAir2")
        self.gobelet1 = Gobelet("Gobelet1", 'V')
        self.gobelet2 = Gobelet("Gobelet2", 'V')
        self.gobelet3 = Gobelet("Gobelet3", 'R')
        self.gobelet4 = Gobelet("Gobelet4", 'R')
        self.phare = Phare("Phare")
        self.bonPortGros = BonPort("BonPortGros")
        self.bonPortPetit = BonPort("BonPortPetit")
        self.pavillon = Pavillon("Pavillon")
        self.todoList = [self.phare.name, self.mancheAir1.name, self.mancheAir2.name, self.gobelet1.name, self.gobelet2.name, self.gobelet3.name, self.gobelet4.name]
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

    def release(self, action):
        self.wipList.remove(action)
        self.todoList.append(action)
        self.updateScore()

    def finish(self, action):
        self.wipList.remove(action)
        self.doneList.append(action)
        self.updateScore()
