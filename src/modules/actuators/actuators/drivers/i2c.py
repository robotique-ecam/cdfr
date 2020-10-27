#!/usr/bin/env python3


"""I2C Base driver for actuators."""


from threading import RLock

try:
    from smbus import SMBus
except ImportError:
    print('[!] Actuators are in simulation mode. No python3-smbus was found !')
    SMBus = int


class I2CDriver:
    """Thread-safe driver interacting over I2C with actuators boards."""

    def __init__(self, bus_id):
        """Create driver with default values."""
        self._mutex = RLock()
        self._bus_id = bus_id
        self._i2c = SMBus(self._bus_id)

    def read_byte(self, addr: int):
        """Read pump driver outputs state."""
        if self._i2c != self._bus_id:
            with self.__mutex__:
                return self._i2c.read_byte(addr)
        return 0

    def write_byte(self, addr: int, value: int):
        """Write pump driver outputs state."""
        if self._i2c != self._bus_id:
            with self.mutex:
                self._i2c.write_byte(addr, value)
