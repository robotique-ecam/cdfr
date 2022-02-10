#!/usr/bin/env python3
from strategix.action_objects import Action, Phare, Ecueil, MancheAir, Gobelet, Chenal, CarreFouille

actions = {
    "CARRE_FOUILLE": CarreFouille(tags={"ONLY_ROBOT": "asterix"}),

    "PAVILLON": Action(tags={"STATUS": "PREEMPT"}),
    "CHENAL": Chenal(position=(1.5, 0.15), tags={"STATUS": "PREEMPT"}),
    "PHARE_BLEU": Phare(
        position=(0.26, 2), tags={"ONLY_ROBOT": "obelix", "ONLY_SIDE": "blue"}
    ),
    "PHARE_JAUNE": Phare(
        position=(2.775, 2), tags={"ONLY_ROBOT": "obelix", "ONLY_SIDE": "yellow"}
    ),
    "MANCHE1": MancheAir(
        position=(0.15, 0.18), tags={"ONLY_SIDE": "blue", "ONLY_ROBOT": "obelix"}
    )
}