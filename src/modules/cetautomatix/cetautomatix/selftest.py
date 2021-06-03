#!/usr/bin/env python3


"""Selftest."""


from time import sleep
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
        self.lcd_driver = self.node.create_publisher(Lcd, "lcd", 1)
        # Wait for subscribers
        while self.lcd_driver.get_subscription_count() < 1:
            sleep(1)

        self.__write__(0)  # Report selftest init
        if "aarch64" not in machine():
            return

        self.__write__(0x10)  # Started Hardware tests

        # Testing for CPU Throttling
        vcgm = Vcgencmd()
        self.__write__(0x11)
        if int(vcgm.get_throttled().get("raw_data"), 16) != 0:
            return

        # Testing devices on sensors I2C Bus
        try:
            bus = SMBus(4)
            for addr in [0x30, 0x31, 0x32, 0x35, 0x36, 0x37]:
                self.__write__(addr)
                bus.read_byte(addr)
        except BaseException:
            return

        # Testing devices on actuators I2C Bus
        code = 0x40
        self.__write__(code)
        try:
            bus = SMBus(3)
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
        for servo in self.node.actuators.SERVOS:
            servo = self.node.actuators.SERVOS.get(servo)
            code += 1
            self.__write__(code)
            if self.node.actuators.arbotix.getPosition(servo.get("addr")) == -1:
                return

        self.__write__(0xFF)

    def __write__(self, code):
        """Write current testing code to LCD."""
        lcd_msg = Lcd()
        lcd_msg.line = 1
        lcd_msg.text = f"Selftest: 0x{hex(code)[2:].zfill(2).upper()}"
        self.lcd_driver.publish(lcd_msg)
