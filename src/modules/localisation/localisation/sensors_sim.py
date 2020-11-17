#!/usr/bin/env python3


"""Simulated sensors definition for localisation."""


class Sensors:
    def __init__(self, node, addrs=[]):
        """Init sensors baseclass."""
        self.node = node
        self._addrs = addrs
        self._vlx_array = []
        for addr in addrs:
            vlx = self.node.robot.getDistanceSensor(f'vlx_0x{addr:x}')
            try:
                vlx.enable(100)  # Sampling rate of 100ms
                self._vlx_array.append(vlx)
            except BaseException:
                print(f'No simulated VL53L1X sensor at address {addr} !')

    def get_distances(self):
        """Fetch distances from VL53L1Xs."""
        distances = {}
        for vlx, addr in zip(self._vlx_array, self._addrs):
            distances.update({addr: round(vlx.getValue(), 4)})
        return distances

    def __del__(self):
        """Destructor."""
        for vlx in self._vlx_array:
            vlx.disable()
