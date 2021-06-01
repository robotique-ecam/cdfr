#!/usr/bin/env python3


""" Sensors definition for localisation """


from VL53L1X import VL53L1X, VL53L1xUserRoi

vlx_order = [30, 31, 32, 33, 34, 35]
"""
30: side_right
31: front_right
32: front_left
33: side_left
34: back_left
35: back_right
"""


class Sensors:
    def __init__(self, i2c_bus=4, addrs=[]):
        """Init Sensors"""
        self._i2c_bus = i2c_bus
        self._vlx_array = []
        for addr in addrs:
            self.init_vlx(addr)

    def init_vlx(self, addr):
        """Init the vlx at the given address"""
        vlx = VL53L1X(i2c_bus=self._i2c_bus, i2c_address=addr)
        try:
            vlx.open()
            vlx.change_address(0x29)
            vlx.close()

            vlx = VL53L1X(i2c_bus=self._i2c_bus, i2c_address=0x29)
            vlx.open(reset=True)
            vlx.change_address(addr)
            vlx.close()

            vlx = VL53L1X(i2c_bus=self._i2c_bus, i2c_address=addr)
            vlx.open()
            vlx.set_inter_measurement_period(38)
            vlx.set_timing_budget(35000)
            vlx.start_ranging(mode=1)
            roi = VL53L1xUserRoi(0, 11, 3, 6)
            vlx.set_user_roi(roi)

            self._vlx_array.append(vlx)
        except BaseException:
            print(f"No VL53L1X sensor on I2C bus {self._i2c_bus} at address {addr} !")

    def get_distances(self):
        """Fetch distances from VL53L1X"""
        distances = {}
        for i in range(len(self._vlx_array)):
            distances.update({vlx_order[i]: self._vlx_array[i].get_distance()})
        return distances

    def __del__(self):
        """Destructor"""
        for vlx in self._vlx_array:
            vlx.stop_ranging()
            vlx.close()
