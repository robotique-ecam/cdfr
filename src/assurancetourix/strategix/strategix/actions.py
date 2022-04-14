#!/usr/bin/env python3
from strategix.action_objects import Action, Phare, Ecueil, MancheAir, Gobelet, Chenal, CarreFouille, Echantillon, Distributeur

actions = {
    "CARRE_FOUILLE": CarreFouille(tags={"ONLY_ROBOT": "asterix"}),
    
    # Border Echantillon
    "ECHANTILLON1": Echantillon(
        position=(-0.06495, 1.7), rotation=180, color="green", tags={"ONLY_ROBOT": "asterix", "ONLY_SIDE": "blue"}
    ),
    "ECHANTILLON2": Echantillon(
        position=(0.1206, 0.3116), rotation=-135, color="blue", tags={"ONLY_ROBOT": "asterix", "ONLY_SIDE": "blue"}
    ),
    "ECHANTILLON3": Echantillon(
        position=(0.3116, 0.1206), rotation=-135, color="red", tags={"ONLY_ROBOT": "asterix", "ONLY_SIDE": "blue"}
    ),
    "ECHANTILLON4": Echantillon(
        position=(3.06495, 1.7), rotation=0, color="green", tags={"ONLY_ROBOT": "asterix", "ONLY_SIDE": "yellow"}
    ),
    "ECHANTILLON5": Echantillon(
        position=(2.6884, 0.1206), rotation=-45, color="red", tags={"ONLY_ROBOT": "asterix", "ONLY_SIDE": "yellow"}
    ),
    "ECHANTILLON6": Echantillon(
        position=(2.8794, 0.3116), rotation=-45, color="blue", tags={"ONLY_ROBOT": "asterix", "ONLY_SIDE": "yellow"}
    ),
    
    # Field Echantillon
    "ECHANTILLON7": Echantillon(
        position=(0.9, 1.445), color="blue", tags={"ONLY_ROBOT": "obelix"}
    ),
    "ECHANTILLON8": Echantillon(
        position=(0.83, 1.325), color="green", tags={"ONLY_ROBOT": "obelix"}
    ),
    "ECHANTILLON9": Echantillon(
        position=(0.9, 1.205), color="red", tags={"ONLY_ROBOT": "obelix"}
    ),
    "ECHANTILLON10": Echantillon(
        position=(2.1, 1.445), color="blue", tags={"ONLY_ROBOT": "obelix"}
    ),
    "ECHANTILLON11": Echantillon(
        position=(2.17, 1.325), color="green", tags={"ONLY_ROBOT": "obelix"}
    ),
    "ECHANTILLON12": Echantillon(
        position=(2.1, 1.205), color="red", tags={"ONLY_ROBOT": "obelix"}
    ),
    
    "DISTRIBUTEUR1": Distributeur(
        position=(0, 1.25), rotation=0, tags={"ONLY_ROBOT": "obelix", "ONLY_SIDE": "blue"}
    ),
    "DISTRIBUTEUR2": Distributeur(
        position=(1.35, 2), rotation=-90, tags={"ONLY_ROBOT": "obelix"}
    ),
    "DISTRIBUTEUR3": Distributeur(
        position=(1.65, 2), rotation=-90, tags={"ONLY_ROBOT": "obelix"}
    ),
    "DISTRIBUTEUR4": Distributeur(
        position=(3, 1.25), rotation=180, tags={"ONLY_ROBOT": "obelix", "ONLY_SIDE": "yellow"}
    ),

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