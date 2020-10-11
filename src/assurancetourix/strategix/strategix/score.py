from cetautomatix.magic_points import GREEN_CUPS, RED_CUPS, elements
from strategix.actions import Ecueil, Gobelet, MancheAir, Pavillon, Phare


class Score:
    def __init__(self):
        self.score = 0
        # self.bonPortGros = BonPort('BonPortGros')
        # self.bonPortPetit = BonPort('BonPortPetit')
        self.actions = {'MANCHE1': MancheAir('MANCHE1'), 'MANCHE2': MancheAir('MANCHE2'),
                        'MANCHE3': MancheAir('MANCHE3'), 'MANCHE4': MancheAir('MANCHE4'),
                        'PHARE_BLEU': Phare('PHARE_BLEU'), 'PHARE_JAUNE': Phare('PHARE_JAUNE'),
                        'PAVILLON': Pavillon('PAVILLON'),
                        'ECUEIL_1': Ecueil('ECUEIL_1', ['GOB35', 'GOB36', 'GOB37', 'GOB38', 'GOB39']),
                        'ECUEIL_2': Ecueil('ECUEIL_2', ['GOB40', 'GOB41', 'GOB42', 'GOB43', 'GOB44']),
                        'ECUEIL_JAUNE': Ecueil('ECUEIL_JAUNE', ['GOB30', 'GOB31', 'GOB32', 'GOB33', 'GOB34']),
                        'ECUEIL_BLEU': Ecueil('ECUEIL_BLEU', ['GOB25', 'GOB26', 'GOB27', 'GOB28', 'GOB29'])
                        }
        self.actions.update({cup: Gobelet(cup, 'R') for cup in list(RED_CUPS.keys())})
        self.actions.update({cup: Gobelet(cup, 'G') for cup in list(GREEN_CUPS.keys())})
        self.excludeFromBlue = ['PHARE_JAUNE', 'MANCHE3', 'MANCHE4', 'ECUEIL_JAUNE']
        self.excludeFromYellow = ['PHARE_BLEU', 'MANCHE1', 'MANCHE2', 'ECUEIL_BLEU']
        self.todoList = list(elements.keys())
        self.wipList = ['PAVILLON']
        self.doneList = []

    def updateScore(self):
        self.score = 0
        # Manche à Air
        numMancheAir = len([action for action in self.doneList if 'MANCHE' in action])
        self.score += 5 if numMancheAir == 1 else 15 if numMancheAir == 2 else 0
        # Gobelets
        gobeletsInBase = [action for action in self.doneList if 'GOB' in action]
        numRedCups = len([self.actions[gob] for gob in gobeletsInBase if self.actions[gob].inChenal and self.actions[gob].color == 'R'])
        numGreenCups = len([self.actions[gob] for gob in gobeletsInBase if self.actions[gob].inChenal and self.actions[gob].color == 'G'])
        pair = 2 * numRedCups if numRedCups < numGreenCups else 2 * numGreenCups
        self.score += len(gobeletsInBase) + numRedCups + numGreenCups + pair
        # Phare
        self.score += 15 if 'PHARE_BLEU' or 'PHARE_JAUNE' in self.doneList else 2
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
        self.score += 10 if 'PAVILLON' in self.doneList else 0

    def preempt(self, action, sender):
        self.actions[action].executer = sender
        self.todoList.remove(action)
        self.wipList.append(action)
        return True

    def release(self, action, sender):
        self.actions[action].executer = None
        self.wipList.remove(action)
        # Don't add failing action again
        return True

    def finish(self, action, sender):
        self.wipList.remove(action)
        self.doneList.append(self.ecueil_to_gob(action))
        return True

    def ecueil_to_gob(self, action):
        new_action = self.actions[action]
        if isinstance(new_action, Ecueil):
            return new_action.gobelets
        else:
            return action
