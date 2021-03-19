#!/usr/bin/env python3


"""Strategix action server and score counter."""

from threading import Thread

import rclpy
from lcd_msgs.msg import Lcd
from std_msgs.msg import UInt8
from rcl_interfaces.msg import SetParametersResult
from rclpy.node import Node
from strategix.actions import actions
from strategix.exceptions import MatchStartedException
from strategix_msgs.srv import ChangeActionStatus, GetAvailableActions


class StrategixActionServer(Node):
    def __init__(self):
        super().__init__("strategix_action_server")
        self.side = self.declare_parameter("side", "blue")
        self.add_on_set_parameters_callback(self._on_set_parameters)
        self.todo_srv = self.create_service(
            GetAvailableActions, "/strategix/available", self.available_callback
        )
        self.action_srv = self.create_service(
            ChangeActionStatus, "/strategix/action", self.action_callback
        )
        self.lcd_driver = self.create_publisher(Lcd, "/obelix/lcd", 1)
        self.score_publisher = self.create_publisher(UInt8, "/score", 1)
        self.get_logger().info(f"Default side is {self.side.value}")
        self.get_logger().info("Strategix is ready")

    def _on_set_parameters(self, params):
        """Handle Parameter events especially for side."""
        result = SetParametersResult()
        try:
            for param in params:
                if param.name == "side":
                    self.get_logger().warn(f"Side changed {param.value}")
                    self.side = param
                else:
                    setattr(self, param.name, param)
            result.successful = True
        except MatchStartedException as e:
            result.reason = e
        return result

    def available_callback(self, request, response):
        """Callback function when a robot needs the list of available actions"""
        self.get_logger().info(f"GET {request.sender}")
        available = []
        for action_id, action_object in actions.items():
            if action_object.tags.get("STATUS") is None and not action_object.tags.get(
                "IN_ECUEIL"
            ):
                if (
                    action_object.tags.get("ONLY_SIDE") is None
                    or action_object.tags.get("ONLY_SIDE") == self.side.value
                ):
                    if (
                        action_object.tags.get("ONLY_ROBOT") is None
                        or action_object.tags.get("ONLY_ROBOT") == request.sender
                    ):
                        available.append(action_id)
        response.available = available
        self.get_logger().info(f"AVAILABLE: {response.available}")
        return response

    def action_callback(self, request, response):
        """Callback function when a robot has to change its current action"""
        try:
            self.get_logger().info(
                f"{request.sender} {request.request} {request.action}"
            )
            if request.request == "PREEMPT":
                response.success = self.preempt(request.action, request.sender)
            elif request.request == "DROP":
                response.success = self.release(request.action, request.sender)
            elif request.request == "CONFIRM":
                response.success = self.finish(request.action, request.sender)
                # Detach update_score coroutine
                Thread(target=self.update_score).start()
            else:
                raise BaseException
        except BaseException:
            self.get_logger().warn(
                f"Invalid call : {request.sender} {request.request} {request.action}"
            )
            response.success = False
        return response

    def update_score(self):
        """Update global score."""
        score = self.get_score()
        lcd_msg = Lcd()
        lcd_msg.line = 1
        lcd_msg.text = f"Score: {score}"
        score_msg = UInt8()
        score_msg.data = score
        self.score_publisher.publish(score_msg)
        self.lcd_driver.publish(lcd_msg)

    def get_score(self):
        score = 0
        num_manche_air = 0
        num_red_cups = 0
        num_green_cups = 0
        phare_raised = False
        pavillon_raised = False
        for action_id, action_object in actions.items():
            if action_object.tags.get("STATUS") == "FINISHED":
                if "MANCHE" in action_id:
                    num_manche_air += 1
                if "GOB" in action_id:
                    if action_object.color == "RED":
                        num_red_cups += 1
                    else:
                        num_green_cups += 1
                if "PHARE" in action_id:
                    phare_raised = True
                if "PAVILLON" in action_id:
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
        score += 15 if num_manche_air == 2 else 5 if num_manche_air == 1 else 0
        pair = 2 * num_red_cups if num_red_cups < num_green_cups else 2 * num_green_cups
        score += 2 * (num_red_cups + num_green_cups) + pair
        score += 15 if phare_raised else 2
        score += 10 if pavillon_raised else 0
        return score

    def preempt(self, action, sender):
        actions.get(action).tags["STATUS"] = "PREEMPT"
        actions.get(action).tags["EXECUTER"] = sender
        # if "ECUEIL" in action:
        #     for gob in actions[action]["GOBS"]:
        #         actions[action]["STATUS"] = "PREEMPTED"
        #         actions[gob]["EXECUTER"] = sender
        return True

    def release(self, action, sender):
        actions.get(action).tags["STATUS"] = "RELEASE"
        actions.get(action).tags["EXECUTER"] = sender
        # if "ECUEIL" in action:
        #     for gob in actions[action]["GOBS"]:
        #         actions[action]["STATUS"] = "RELEASED"
        #         actions[gob]["EXECUTER"] = None
        # Don't add failing action again
        return True

    def finish(self, action, sender):
        actions.get(action).tags["STATUS"] = "FINISH"
        actions.get(action).tags["EXECUTER"] = sender
        # if "ECUEIL" in action:
        #     for gob in actions[action]["GOBS"]:
        #         actions[action]["STATUS"] = "FINISHED"
        #         actions[gob]["EXECUTER"] = sender
        return True


def main(args=None):
    rclpy.init(args=args)
    strategix = StrategixActionServer()
    try:
        rclpy.spin(strategix)
    except KeyboardInterrupt:
        pass
    strategix.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
