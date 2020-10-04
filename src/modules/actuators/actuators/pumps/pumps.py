#!/usr/bin/env python3


"""Pumps Hardware Interface."""


from smbus import SMBus


class PumpDriver:

    def __init__(self, bus_id=1, addrs=[0x40]):
        self._i2c = SMBus(bus_id)
        self._addrs = addrs
        self._len = len(self._addrs)

    def __read__(self, addr: int):
        """Read pump driver outputs state."""
        self._i2c.read_byte(addr)

    def __write__(self, addr: int, value: int):
        """Write pump driver outputs state."""
        self._i2c.write_byte(addr, value)

    def _get_state(self):
        """Get outputs state from all drivers."""
        values = bytes(self._len)
        for addr, i in zip(self._addrs, range(self._len)):
            values[i] = self.__read__(addr)
        return int.from_bytes(values, 'big')

    def _set_state(self, value: int):
        """Set state for all drivers."""
        values = value.to_bytes(self._len, byteorder='big')
        for addr, byte in zip(self._addrs, values):
            self.__write__(addr, byte)

    def bytes_set(self, values: list):
        """Set list of addresses in argument to 1."""
        state, new_state = self._get_state(), 0
        for addr in values:
            new_state += (1 << addr)
        self._set_state(new_state | state)

    def bytes_clear(self, values: list):
        """Set list of addresses in argument to 0."""
        state, new_state = self._get_state(), 0
        for addr in values:
            new_state += (1 << addr)
        mask_size, mask = len(self._addrs), 0
        for i in range(mask_size):
            mask += 0xFF << (8 * i)
        self._set_state(state & (~(new_state) & mask))
