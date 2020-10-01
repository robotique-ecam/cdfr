#!/usr/bin/env python3


"""Panoramix ROS node."""


import re

import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import Log


class Panoramix(Node):
    """Panoramix ROS Node."""

    def __init__(self, socketio, namespace):
        """Construct."""
        super().__init__('panoramix_node')
        self._socketio = socketio
        self._namespace = namespace
        self._ansi_escape = re.compile(r'(?:\x1B[@-_]|[\x80-\x9F])[0-?]*[ -/]*[@-~]')
        self._subscriptions = []
        for ns in ['', '/asterix', '/obelix', '/assurancetourix']:
            self._subscriptions.append(self.create_subscription(Log, f'{ns}/rosout', self.log_callback, 5))

    def _escape_ansi(self, txt):
        """Return ANSI escaped text."""
        escaped = self._ansi_escape.sub('', txt)
        if self._ansi_escape.match(txt):
            return f'<b>{escaped}</b>'
        return escaped

    def _loglevel(self, level):
        """Loglevel."""
        level = bytes([level])
        if level == Log.FATAL:
            return 'FATAL'
        elif level == Log.ERROR:
            return 'ERROR'
        elif level == Log.WARN:
            return 'WARN'
        elif level == Log.INFO:
            return 'INFO'
        elif level == Log.DEBUG:
            return 'DEBUG'

    def log_callback(self, msg):
        """Callback for /rosout."""
        loglevel = self._loglevel(msg.level)
        message = self._escape_ansi(msg.msg)
        self._socketio.emit(
            'console_out',
            {'text': f'[{loglevel}] [{msg.name}] {message}<br>'},
            namespace=self._namespace,
        )
        if loglevel in ('WARN', 'ERROR', 'FATAL'):
            self._socketio.emit(
                'console_msg',
                {'level': loglevel, 'name': msg.name, 'msg': message},
                namespace=self._namespace,
            )


def ros_events_thread(socketio=None, namespace=None, args=None):
    """Wrap for server side generated events."""
    rclpy.init(args=args)
    panoramix = Panoramix(socketio, namespace)
    rclpy.spin(panoramix)
    panoramix.destroy_node()
    rclpy.shutdown()
