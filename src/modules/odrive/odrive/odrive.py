#!/usr/bin/env python3


from time import sleep
from math import pi, sin, cos

import odrive
import odrive.enums as odrv
from rclpy.node import Node


class Odrive(Node):
    """Odrive interface."""

    def __init__(self, simulated=False):
        super().__init__(self, 'odrive_node')
        self.simulated = simulated
        if not self.simulated:
            self.odrive = odrive.find_any()
            self.get_logger().info('Connected to oDrive')
        else:
            self.get_logger().info('Simulating oDrive')
        self.setup()

    def get_version(self):
        """Get version from odrive."""
        version_string = ''
        if not self.simulated:
            with self.odrive as o:
                version_string += f'Hardware Version {o.hw_version_major}.{o.hw_version_minor}.{o.hw_version_variant}\n'
                version_string += f'Firmware version {o.fw_version_major}.{o.fw_version_minor}.{o.fw_version_revision}\n'
        else:
            version_string += 'Simulation Version 0.1'
        version_string += f'API Version {odrive.version.get_version_str()}'
        return version_string

    def motor_calibration(self):
        """Do motor calibration on both wheels."""
        if not self.simulated:
            self.odrive.axis0.requested_state = self.odrive.axis1.requested_state = odrv.AXIS_STATE_FULL_CALIBRATION_SEQUENCE
            while self.odrive.axis0.requested_state != odrv.AXIS_STATE_IDLE or self.odrive.axis1.requested_state != odrv.AXIS_STATE_IDLE:
                sleep(.1)
        self.get_logger().info('Motor calibration is done')

    def setup(self):
        """Setup both motors for CLC."""
        if not self.simulated:
            self.odrive.axis0.requested_state = self.odrive.axis1.requested_state = odrv.AXIS_STATE_CLOSED_LOOP_CONTROL

    def update_odometry(self):
        """Update odometry from axis encoders."""
        self.odrive.axis0.encoder
