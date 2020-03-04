#!/usr/bin/env python3


"""Pumps Hardware Interface."""


from smbus import SMBus


class PumpDriver:

    def __init__(self, bus_id=1, addrs=[0x40]):
        self._i2c = SMBus(bus_id)
        self._addrs = addrs
        self._len = len(self._addrs)

    def __read__(self, addr):
        """Read pump driver outputs state."""
        self._i2c.read_byte(addr)

    def __write__(self, addr, value):
        """Write pump driver outputs state."""
        self._i2c.write_byte(addr, value)

    def get_state(self):
        """Get outputs state from all drivers."""
        values = bytes(self._len)
        for addr, i in zip(self._addrs, range(self._len)):
            values[i] = self.__read__(addr)
        return int.from_bytes(values, 'big')

    def set_state(self, value):
        """Set state for all drivers."""
        values = value.to_bytes(self._len, byteorder='big')
        for addr, byte in zip(self._addrs, values):
            self.__write__(addr, byte)
