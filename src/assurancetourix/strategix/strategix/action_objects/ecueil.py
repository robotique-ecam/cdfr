from .action import Action


class Ecueil(Action):
    def __init__(self, position, **kwargs):
        super().__init__(position, **kwargs)
        self.gob_list = kwargs.get("gob_list", [])

    def get_initial_position(self, robot):
        return self.position

    # def preempt_action(self):
    #     for gobelet_id in self.gob_list:
    #         gobelet = actions.get(gobelet_id)
    #         gobelet.preempt_action()

    # def release_action(self):
    #     for gobelet_id in self.gob_list:
    #         gobelet = actions.get(gobelet_id)
    #         gobelet.release_action()

    # def finish_action(self):
    #     for gobelet_id in self.gob_list:
    #         gobelet = actions.get(gobelet_id)
    #         gobelet.finish_action()
