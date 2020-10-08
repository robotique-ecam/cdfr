#!/usr/bin/env python3

# Copyright (c) 2019-2020 ECAM Makers.
# Copyright (c) 2008-2019 Vanadium Labs LLC.
# All right reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#   * Redistributions of source code must retain the above copyright
#     notice, this list of conditions and the following disclaimer.
#   * Redistributions in binary form must reproduce the above copyright
#     notice, this list of conditions and the following disclaimer in the
#     documentation and/or other materials provided with the distribution.
#   * Neither the name of Vanadium Labs LLC nor the names of its
#     contributors may be used to endorse or promote products derived
#     from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
# ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
# WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL VANADIUM LABS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
# LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA,
# OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
# LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE
# OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF
# ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

# Author: Michael Ferguson

# @file arbotix.py Low-level code to control an ArbotiX.

from threading import RLock

import serial

from actuators.arbotix import ax12

# @brief ArbotiX errors. Used by now to handle broken serial connections.


class ArbotiXException(Exception):
    pass

# @brief This class controls an ArbotiX, USBDynamixel, or similar board through a serial connection.


class ArbotiX:

    # @brief Constructs an ArbotiX instance, optionally opening the serial connection.
    ##
    # @param port The name of the serial port to open.
    ##
    # @param baud The baud rate to run the port at.
    ##
    # @param timeout The timeout to use for the port. When operating over a wireless link, you may need to
    # increase this.
    ##
    # @param open Whether to open immediately the serial port.
    def __init__(self, port="/dev/ttyUSB0", baud=115200, interpolation=False, direct=False):
        """ This may throw errors up the line -- that's a good thing. """
        self.ser = serial.Serial()
        self.ser.baudrate = baud
        self.ser.port = port
        self.ser.timeout = 0.5
        self.ser.open()
        self.error = 0
        self.hasInterpolation = interpolation
        self.direct = direct

    def execute(self, index, ins, params):
        """ Send an instruction to a device."""
        self.ser.flushInput()
        packet = bytes()
        length = 2 + len(params)
        checksum = 255 - ((index + length + ins + sum(params)) % 256)
        packet += bytes([0xFF, 0xFF, index, length, ins])
        for val in params:
            packet += bytes([val])
        packet += bytes([checksum])
        self.ser.write(packet)
        return self.getPacket(0)

    def setReg(self, index, regstart, values):
        """ Set the value of registers. Should be called as such:
        ax12.setReg(1,1,(0x01,0x05)) """
        self.execute(index, ax12.AX_WRITE_DATA, [regstart] + values)
        return self.error

    def getPacket(self, mode, id=-1, leng=-1, error=-1, params=None):
        """ Read a return packet, iterative attempt """
        # need a positive byte
        b = self.ser.read()
        if b == b'':
            print("Fail Read")
            return None

        d = b[0]
        # now process our byte
        if mode == 0:           # get our first 0xFF
            if d == 0xff:
                return self.getPacket(1)
            else:
                return self.getPacket(0)
        elif mode == 1:         # get our second 0xFF
            if d == 0xff:
                return self.getPacket(2)
            else:
                return self.getPacket(0)
        elif mode == 2:         # get id
            if d != 0xff:
                return self.getPacket(3, d)
            else:
                return self.getPacket(0)
        elif mode == 3:         # get length
            return self.getPacket(4, id, d)
        elif mode == 4:         # read error
            self.error = d
            if leng == 2:
                return self.getPacket(6, id, leng, d, [])
            else:
                return self.getPacket(5, id, leng, d, [])
        elif mode == 5:         # read params
            params.append(d)
            if len(params) + 2 == leng:
                return self.getPacket(6, id, leng, error, params)
            else:
                return self.getPacket(5, id, leng, error, params)
        elif mode == 6:         # read checksum
            checksum = id + leng + error + sum(params) + d
            if checksum % 256 != 255:
                return None
            return params
        # fail
        return None

    def getReg(self, index, regstart, rlength):
        """Get the value of registers, should be called as such:
        ax12.getReg(1,1,1)."""
        vals = self.execute(index, ax12.AX_READ_DATA, [regstart, rlength])
        if vals is None:
            return -1
        if rlength == 1:
            return vals[0]
        return vals

    def syncWrite(self, regstart, vals):
        """Set the value of registers. Should be called as such:
        ax12.syncWrite(reg, ((id1, val1, val2), (id2, val1, val2))) """
        self.ser.flushInput()
        length = 4
        valsum = 0
        packet = ""
        for i in vals:
            length = length + len(i)
            valsum = valsum + sum(i)
        checksum = 255 - ((254 + length + ax12.AX_SYNC_WRITE +
                           regstart + len(vals[0]) - 1 + valsum) % 256)
        # packet: FF FF ID LENGTH INS(0x03) PARAM .. CHECKSUM
        packet += chr(0xFF) + chr(0xFF) + chr(0xFE) + chr(length) + chr(ax12.AX_SYNC_WRITE) + chr(regstart) + chr(len(vals[0]) - 1)
        for servo in vals:
            for value in servo:
                packet += chr(value)
        packet += chr(checksum)
        self.ser.write(packet.encode())
        # no return info...

    # @brief Read values of registers on many servos.
    ##
    # @param servos A list of the servo IDs to read from.
    ##
    # @param start The starting register address to begin reading at.
    ##
    # @param length The number of bytes to read from each servo.
    ##
    # @return A list of bytes read.
    def syncRead(self, servos, start, length):
        return self.execute(0xFE, ax12.AX_SYNC_READ, [start, length] + servos)

    # @brief Set baud rate of a device.
    ##
    # @param index The ID of the device to write (Note: ArbotiX is 253).
    ##
    # @param baud The baud rate.
    ##
    # @return The error level.
    def setBaud(self, index, baud):
        return self.write(index, ax12.P_BAUD_RATE, [baud, ])

    # @brief Get the return level of a device.
    ##
    # @param index The ID of the device to read.
    ##
    # @return The return level, .
    def getReturnLevel(self, index):
        try:
            return int(self.read(index, ax12.P_RETURN_LEVEL, 1)[0])
        except OSError:
            return -1

    # @brief Set the return level of a device.
    ##
    # @param index The ID of the device to write.
    ##
    # @param value The return level.
    ##
    # @return The error level.
    def setReturnLevel(self, index, value):
        return self.write(index, ax12.P_RETURN_LEVEL, [value])

    # @brief Turn on the torque of a servo.
    ##
    # @param index The ID of the device to enable.
    ##
    # @return The error level.
    def enableTorque(self, index):
        return self.write(index, ax12.P_TORQUE_ENABLE, [1])

    # @brief Turn on the torque of a servo.
    ##
    # @param index The ID of the device to disable.
    ##
    # @return The error level.
    def disableTorque(self, index):
        return self.write(index, ax12.P_TORQUE_ENABLE, [0])

    # @brief Set the status of the LED on a servo.
    ##
    # @param index The ID of the device to write.
    ##
    # @param value 0 to turn the LED off, >0 to turn it on
    ##
    # @return The error level.
    def setLed(self, index, value):
        return self.write(index, ax12.P_LED, [value])

    # @brief Set the position of a servo.
    ##
    # @param index The ID of the device to write.
    ##
    # @param value The position to go to in, in servo ticks.
    ##
    # @return The error level.
    def setPosition(self, index, value):
        return self.write(index, ax12.P_GOAL_POSITION_L, [value % 256, value >> 8])

    # @brief Set the speed of a servo.
    ##
    # @param index The ID of the device to write.
    ##
    # @param value The speed to write.
    ##
    # @return The error level.
    def setSpeed(self, index, value):
        return self.write(index, ax12.P_GOAL_SPEED_L, [value % 256, value >> 8])

    # @brief Get the position of a servo.
    ##
    # @param index The ID of the device to read.
    ##
    # @return The servo position.
    def getPosition(self, index):
        values = self.read(index, ax12.P_PRESENT_POSITION_L, 2)
        try:
            return int(values[0]) + (int(values[1]) << 8)
        except OSError:
            return -1

    # @brief Get the speed of a servo.
    ##
    # @param index The ID of the device to read.
    ##
    # @return The servo speed.
    def getSpeed(self, index):
        values = self.read(index, ax12.P_PRESENT_SPEED_L, 2)
        try:
            return int(values[0]) + (int(values[1]) << 8)
        except OSError:
            return -1

    # @brief Get the goal speed of a servo.
    ##
    # @param index The ID of the device to read.
    ##
    # @return The servo goal speed.
    def getGoalSpeed(self, index):
        values = self.read(index, ax12.P_GOAL_SPEED_L, 2)
        try:
            return int(values[0]) + (int(values[1]) << 8)
        except OSError:
            return -1

    # @brief Get the voltage of a device.
    ##
    # @param index The ID of the device to read.
    ##
    # @return The voltage, in Volts.
    def getVoltage(self, index):
        try:
            return int(self.read(index, ax12.P_PRESENT_VOLTAGE, 1)[0]) / 10.0
        except OSError:
            return -1

    # @brief Get the temperature of a device.
    ##
    # @param index The ID of the device to read.
    ##
    # @return The temperature, in degrees C.
    def getTemperature(self, index):
        try:
            return int(self.read(index, ax12.P_PRESENT_TEMPERATURE, 1)[0])
        except OSError:
            return -1

    # @brief Determine if a device is moving.
    ##
    # @param index The ID of the device to read.
    ##
    # @return True if servo is moving.
    def isMoving(self, index):
        try:
            d = self.read(index, ax12.P_MOVING, 1)[0]
        except OSError:
            return True
        return d != 0

    # @brief Put a servo into wheel mode (continuous rotation).
    ##
    # @param index The ID of the device to write.
    def enableWheelMode(self, index):
        self.write(index, ax12.P_CCW_ANGLE_LIMIT_L, [0, 0])

    # @brief Put a servo into servo mode.
    ##
    # @param index The ID of the device to write.
    ##
    # @param resolution The resolution of the encoder on the servo. NOTE: if using
    # 12-bit resolution servos (EX-106, MX-28, etc), you must pass resolution = 12.
    ##
    # @return
    def disableWheelMode(self, index, resolution=10):
        resolution = (2 ** resolution) - 1
        self.write(index, ax12.P_CCW_ANGLE_LIMIT_L, [
                   resolution % 256, resolution >> 8])

    # Direction definition for setWheelSpeed
    FORWARD = 0
    # Direction definition for setWheelSpeed
    BACKWARD = 1

    # @brief Set the speed and direction of a servo which is in wheel mode (continuous rotation).
    ##
    # @param index The ID of the device to write.
    ##
    # @param direction The direction of rotation, either FORWARD or BACKWARD
    ##
    # @param speed The speed to move at (0-1023).
    ##
    # @return
    def setWheelSpeed(self, index, direction, speed):
        if speed > 1023:
            speed = 1023
        if direction == self.FORWARD:
            # 0~1023 is forward, it is stopped by setting to 0 while rotating to CCW direction.
            self.write(index, ax12.P_GOAL_SPEED_L, [speed % 256, speed >> 8])
        else:
            # 1024~2047 is backward, it is stopped by setting to 1024 while rotating to CW direction.
            speed += 1024
            self.write(index, ax12.P_GOAL_SPEED_L, [speed % 256, speed >> 8])

    ###########################################################################
    # Extended ArbotiX Driver

    # Helper definition for analog and digital access.
    LOW = 0
    # Helper definition for analog and digital access.
    HIGH = 0xff
    # Helper definition for analog and digital access.
    INPUT = 0
    # Helper definition for analog and digital access.
    OUTPUT = 0xff

    # ArbotiX-specific register table
    # We do Model, Version, ID, Baud, just like the AX-12
    # Register base address for reading digital ports
    REG_DIGITAL_IN0 = 5
    REG_DIGITAL_IN1 = 6
    REG_DIGITAL_IN2 = 7
    REG_DIGITAL_IN3 = 8
    # Register address for triggering rescan
    REG_RESCAN = 15
    # 16, 17 = RETURN, ALARM
    # Register address of first analog port (read only).
    # Each additional port is BASE + index.
    ANA_BASE = 18
    # Register address of analog servos. Up to 10 servos, each
    # uses 2 bytes (L, then H), pulse width (0, 1000-2000ms) (Write only)
    SERVO_BASE = 26
    # Address 46 is Moving, just like an AX-12
    REG_DIGITAL_OUT0 = 47

    # @brief Force the ArbotiX2 to rescan the Dynamixel busses.
    def rescan(self):
        self.write(253, self.REG_RESCAN, [1, ])

    # @brief Get the value of an analog input pin.
    ##
    # @param index The ID of the pin to read (0 to 7).
    ##
    # @param leng The number of bytes to read (1 or 2).
    ##
    # @return 8-bit/16-bit analog value of the pin, -1 if error.
    def getAnalog(self, index, leng=1):
        try:
            val = self.read(253, self.ANA_BASE + int(index), leng)
            return sum(val[i] << (i * 8) for i in range(leng))
        except OSError:
            return -1

    # @brief Get the value of an digital input pin.
    ##
    # @param index The ID of the pin to read (0 to 31).
    ##
    # @return 0 for low, 255 for high, -1 if error.
    def getDigital(self, index):
        try:
            if index < 32:
                x = self.read(253, self.REG_DIGITAL_IN0 + int(index / 8), 1)[0]
            else:
                return -1
        except OSError:
            return -1
        if x & (2**(index % 8)):
            return 255
        else:
            return 0

    # @brief Get the value of an digital input pin.
    ##
    # @param index The ID of the pin to write (0 to 31).
    ##
    # @param value The value of the port, >0 is high.
    ##
    # @param direction The direction of the port, >0 is output.
    ##
    # @return -1 if error.
    def setDigital(self, index, value, direction=0xff):
        if index > 31:
            return -1
        if value == 0 and direction > 0:
            self.write(253, self.REG_DIGITAL_OUT0 + int(index), [1])
        elif value > 0 and direction > 0:
            self.write(253, self.REG_DIGITAL_OUT0 + int(index), [3])
        elif value > 0 and direction == 0:
            self.write(253, self.REG_DIGITAL_OUT0 + int(index), [2])
        else:
            self.write(253, self.REG_DIGITAL_OUT0 + int(index), [0])
        return 0

    # @brief Set the position of a hobby servo.
    ##
    # @param index The ID of the servo to write (0 to 7).
    ##
    # @param value The position of the servo in milliseconds (1500-2500).
    # A value of 0 disables servo output.
    ##
    # @return -1 if error.
    def setServo(self, index, value):
        if index > 7:
            return -1
        if value != 0 and (value < 500 or value > 2500):
            print('ArbotiX Error: Servo value out of range:', value)
        else:
            self.write(253, self._SERVO_BASE + 2 *
                       index, [value % 256, value >> 8])
        return 0
