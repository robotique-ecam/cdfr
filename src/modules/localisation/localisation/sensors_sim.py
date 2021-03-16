#!/usr/bin/env python3


"""Simulated sensors definition for localisation."""

from os import environ

from webots_ros2_core.webots_node import WebotsNode


class Sensors:
    def __init__(self, node, addrs=[]):
        """Init sensors baseclass."""
        supervisor = node.robot + "_vlx_manager"
        environ["WEBOTS_ROBOT_NAME"] = supervisor
        self.node = VlxSupervisor(supervisor, args=None)
        self._addrs = addrs
        self._vlx_array = []
        for addr in addrs:
            vlx = self.node.robot.getDevice(f"vlx_0x{addr}")
            try:
                vlx.enable(100)  # Sampling rate of 100ms
                self._vlx_array.append(vlx)
            except BaseException:
                print(f"No simulated VL53L1X sensor at address {addr} !")

    def get_distances(self):
        """Fetch distances from VL53L1Xs."""
        distances = {}
        for vlx, addr in zip(self._vlx_array, self._addrs):
            self.node.robot.step(1)
            distances.update({addr: round(vlx.getValue(), 4)})
        return distances

    def __del__(self):
        """Destructor."""
        for vlx in self._vlx_array:
            vlx.disable()


class VlxSupervisor(WebotsNode):
    """Vlx manager node."""

    def __init__(self, supervisor, args=None):
        """Init pharaon sim node."""
        super().__init__(supervisor, args)
