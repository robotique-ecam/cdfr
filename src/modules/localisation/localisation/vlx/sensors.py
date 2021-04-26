#!/usr/bin/env python3


""" Sensors definition for localisation """


from VL53L1X import VL53L1X


class Sensors:
    def __init__(self, i2c_bus=3, addrs=[]):
        """Init Sensors"""
        self._i2c_bus = i2c_bus
        self._vlx_array = []
        for addr in addrs:
            vlx = VL53L1X(i2c_bus=self._i2c_bus, i2c_address=addr)
            try:
                vlx.open()
                vlx.start_ranging(2)  # Medium range
                self._vlx_array.append(vlx)
            except BaseException:
                print(
                    f"No VL53L1X sensor on I2C bus {self._i2c_bus} at address {addr} !"
                )

    def get_distances(self):
        """Fetch distances from VL53L1X"""
        distances = {}
        for vlx in self._vlx_array:
            distances.update({vlx.i2c_address: vlx.get_distance()})
        return distances

    def __del__(self):
        """Destructor"""
        for vlx in self._vlx_array:
            vlx.stop_ranging()
            vlx.close()
