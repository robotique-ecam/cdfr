#!/usr/bin/env python3


"""Shared utils functions between asterix and obelix."""


from numpy import interp
from time import sleep

from actuators.arbotix.arbotix import ArbotiX
from actuators.drivers.i2c import I2CDriver
from actuators.drivers.slider import I2CSliderDriver
from actuators.drivers.pumps import PumpDriver
from actuators.drivers.rpi_servos import RPiServos

NO, NC = False, True


class Actuators:

    """Actuators base class."""

    def __init__(
        self,
        i2c_bus=3,
        pump_addr=[0x40],
        FANS=[7],
        PUMPS={},
        DYNAMIXELS=[],
        SERVOS={},
        i2c_bus_servo_motor=5,
        SERVO_MOTORS={},
    ):
        """."""
        self.FANS = FANS
        self.PUMPS = PUMPS
        self.SERVOS = SERVOS
        self.DYNAMIXELS = DYNAMIXELS
        self.SERVO_MOTORS = SERVO_MOTORS
        self.pump_addr = pump_addr
        self._i2c_bus = I2CDriver(i2c_bus)
        self._i2c_bus_servo = I2CDriver(i2c_bus_servo_motor)
        self.pump_driver = PumpDriver(self._i2c_bus, addrs=self.pump_addr)
        self.slider = I2CSliderDriver(self._i2c_bus)
        if len(self.DYNAMIXELS) > 0:
            self.arbotix = ArbotiX()
            self._setupDynamixels()
        if len(self.SERVO_MOTORS) > 0:
            self.rpi_servos = RPiServos(self._i2c_bus_servo, addrs=self.SERVO_MOTORS)
            self._setupServoMotor()
        self.robot_node = None

    def _setupDynamixels(self):
        """Setup dynamixels speed."""
        for dyna in self.DYNAMIXELS:
            d = self.DYNAMIXELS[dyna]
            self.arbotix.setSpeed(dyna, d.get("speed"))
            self.arbotix.setPosition(dyna, d.get("down"))
        for s in self.SERVOS:
            servo = self.SERVOS[s]
            self.arbotix.setSpeed(servo.get("addr"), servo.get("speed"))
            self.arbotix.setPosition(servo.get("addr"), servo.get("down"))

    def _setupServoMotor(self):
        """setup servo_motor to initial position"""
        self.setServoPosition("arm", 82)
        self.setServoPosition("reef", 0)

    def disableDynamixels(self):
        """Setup dynamixels speed."""
        for dyna in self.DYNAMIXELS:
            self.arbotix.disableTorque(dyna)
        for s in self.SERVOS:
            servo = self.SERVOS[s]
            self.arbotix.disableTorque(servo.get("addr"))

    def raiseTheFlag(self):
        """Raise obelix flags. Servo must be bound with ArbotixM."""
        flag_servo = self.SERVOS.get("flags")
        if flag_servo is not None:
            self.arbotix.setPosition(flag_servo["addr"], flag_servo["up"])
            self.robot_node.get_logger().info("Raise the flag")

    def setPumpsEnabled(self, enabled: bool, pumps: list):
        """Set list of pumps as enabled or not."""
        relax = []
        for p in pumps:
            pump = self.PUMPS.get(p)
            if pump is not None:
                if enabled and pump.get("type") == NC:
                    self.pump_driver.bytes_set([pump.get("pump")])
                elif enabled and pump.get("type") == NO:
                    self.pump_driver.bytes_set([pump.get("pump"), pump.get("valve")])
                elif not enabled and pump.get("type") == NC:
                    self.pump_driver.bytes_clear([pump.get("pump")])
                    self.pump_driver.bytes_set([pump.get("valve")])
                    relax.append(pump.get("valve"))
                    self.pump_driver.bytes_clear([pump.get("valve")])
                elif not enabled and pump.get("type") == NO:
                    self.pump_driver.bytes_clear([pump.get("pump"), pump.get("valve")])
                self.robot_node.get_logger().info(
                    f"{'Enabled' if enabled else 'Disabled'} pump {p}"
                )
        # Relax valves to normal state after 100ms minimum delay
        if len(relax) > 0:
            sleep(0.1)
            self.pump_driver.bytes_clear(relax)

    def asterix_grab(self, id):
        pump = self.PUMPS.get(id)
        self.pump_driver.bytes_clear([pump.get("valve")])
        sleep(0.2)
        self.pump_driver.bytes_set([pump.get("pump")])
        self.pump_driver.bytes_set([pump.get("push")])
        sleep(0.2)
        self.pump_driver.bytes_clear([pump.get("pull")])
        sleep(1)
        self.pump_driver.bytes_set([pump.get("pull")])
        sleep(0.2)
        self.pump_driver.bytes_clear([pump.get("push")])

    def asterix_drop(self, id):
        pump = self.PUMPS.get(id)
        self.pump_driver.bytes_clear([pump.get("pull")])
        sleep(0.2)
        self.pump_driver.bytes_set([pump.get("push")])
        sleep(1)
        self.pump_driver.bytes_clear([pump.get("pump")])
        self.pump_driver.bytes_clear([pump.get("push")])
        sleep(0.2)
        self.pump_driver.bytes_set([pump.get("valve")])
        self.pump_driver.bytes_set([pump.get("pull")])

    def asterix_drop_all(self):
        pulls = [pump.get("pull") for pump in self.PUMPS.values()]
        pushes = [pump.get("push") for pump in self.PUMPS.values()]
        valves = [pump.get("valve") for pump in self.PUMPS.values()]
        pumps = [pump.get("pump") for pump in self.PUMPS.values()]
        self.pump_driver.bytes_clear(pulls)
        sleep(0.2)
        self.pump_driver.bytes_set(pushes)
        sleep(1)
        self.pump_driver.bytes_clear(pumps)
        self.pump_driver.bytes_clear(pushes)
        sleep(0.2)
        self.pump_driver.bytes_set(valves)
        self.pump_driver.bytes_set(pulls)

    def grabCups(self, ports: list):
        """Grab cups at specified indexes."""
        self.setPumpsEnabled(True, ports)
        sleep(0.5)
        positions, servos = [], []
        for servo in ports:
            if servo in self.DYNAMIXELS:
                servos.append(servo)
                positions.append(self.DYNAMIXELS[servo].get("up"))
        self.setDynamixelsPositions(servos, positions)

    def dropCups(self, ports: list):
        """Drop cups at specified indexes."""
        positions, servos = [], []
        for servo in ports:
            if servo in self.DYNAMIXELS:
                servos.append(servo)
                positions.append(self.DYNAMIXELS[servo].get("down"))
        self.setDynamixelsPositions(servos, positions)
        self.setPumpsEnabled(False, ports)

    def setDynamixelsPositions(self, addrs: list, positions: list):
        """Set list of pumps as enabled or not."""
        for addr, position in zip(addrs, positions):
            self.arbotix.setPosition(addr, position)
            self.robot_node.get_logger().info(
                f"Set Dynamixel {addr} at position {position}"
            )

    def setFansEnabled(self, enabled: bool):
        """Set fans on and off."""
        if enabled:
            self.pump_driver.bytes_set(self.FANS)
        else:
            self.pump_driver.bytes_clear(self.FANS)
        self.robot_node.get_logger().info(
            f"{'Enabled' if enabled else 'Disabled'} fans"
        )

    def setServoPosition(self, servo: str, position: int):
        """servo motor position"""
        self.rpi_servos.set_angles(servo, position)

    def setSliderPosition(self, position: int):
        """Slider position"""
        self.slider.set_position(position)
        self.robot_node.get_logger().info(f"Set slider at position {position}")

    def getResistanceColor(self) -> str:
        """Calculate 'carré de fouille' resistance. Servo must be connected to the Arduino Nano."""
        # TODO: Rotate servo
        self.rpi_servos.set_angles("resistance", 10)
        sleep(2)
        self.robot_node.get_logger().info("Deployed resistance reader")
        
        # Convert value from (0, 255) to resistance value
        value = self.rpi_servos.read_value("resistance")
        value = interp(value, (0, 255), (0, 5))
        resistance = value * 2200 / (5 - value)
        
        if abs(resistance - 470) < 100:
            color = "yellow"
        elif abs(resistance - 1000) < 100:
            color = "blue"
        else:
            color = "none"
        self.robot_node.get_logger().info("Resistance color:", color)

        sleep(1)
        self.rpi_servos.set_angles("resistance", 110)
        self.robot_node.get_logger().info("Retracted resistance reader")
        
        return color

