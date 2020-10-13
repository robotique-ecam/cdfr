#!/usr/bin/env python3


"""Selftest."""


from platform import machine

from lcd_msgs.msg import Lcd

try:
    from smbus import SMBus
    from vcgencmd import Vcgencmd
except ImportError:
    pass


class Selftest:
    """Selftest wrapper."""

    def __init__(self, node):
        """Init selftest with node."""
        self.node = node
        self.lcd_driver = self.node.create_publisher(Lcd, 'lcd', 1)
        self.__write__(0)  # Report selftest init
        if 'aarch64' not in machine():
            return

        self.__write__(0x10)  # Started Hardware tests

        # Testing for CPU Throttling
        vcgm = Vcgencmd()
        self.__write__(0x11)
        if vcgm.get_throttled() != 0:
            return

        # Testing devices on Drive I2C Bus
        code = 0x20
        self.__write__(code)
        try:
            bus = SMBus(1)
            for addr in [0x10, 0x11]:
                code += 1
                self.__write__(code)
                bus.write_byte_data(addr, 0x80, 0x10)
        except BaseException:
            return

        # Testing devices on sensors I2C Bus
        code = 0x30
        self.__write__(code)
        try:
            bus = SMBus(3)
            for addr in []:
                code += 1
                self.__write__(code)
        except BaseException:
            return

        # Testing devices on actuators I2C Bus
        code = 0x40
        self.__write__(code)
        try:
            bus = SMBus(1)
            for addr in self.node.actuators.pump_addr:
                code += 1
                self.__write__(code)
                bus.read_byte(addr)
        except BaseException:
            return

        # NOTE : Testing devices on accessories bus (I2C6 is not relevant)
        # due to the LCD being present

        # Testing dynamixels connection
        code = 0x50
        self.__write__(code)
        for dyna in self.node.actuators.DYNAMIXELS:
            code += 1
            self.__write__(code)
            if self.node.actuators.arbotix.getPosition(dyna) == -1:
                return
        for servo in self.node.actuators.DYNAMIXELS:
            servo = self.node.actuators.DYNAMIXELS.get(servo)
            code += 1
            self.__write__(code)
            if self.node.actuators.arbotix.getPosition(servo.get('addr')) == -1:
                return

        self.__write(0xFF)

    def __write__(self, code):
        """Write current testing code to LCD."""
        lcd_msg = Lcd()
        lcd_msg.line = 1
        lcd_msg.text = f'Selftest: 0x{hex(code)[2:].zfill(2).upper()}'
        self.lcd_driver.publish(lcd_msg)
