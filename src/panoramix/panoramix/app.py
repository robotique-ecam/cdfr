#!/usr/bin python3


"""Main Node containing Panoramix managment and monitoring system."""


import re

import eventlet
eventlet.monkey_patch()

from threading import Thread

from flask import Flask, render_template
from flask_socketio import SocketIO

import rclpy
from rcl_interfaces.msg import Log
from std_msgs.msg import UInt8
from rclpy.node import Node



async_mode = None
namespace = '/api'

app = Flask(__name__)
socketio = SocketIO(app, async_mode=async_mode)


class Panoramix(Node):
    """Panoramix ROS Node."""

    def __init__(self, socketio, namespace):
        """Construct."""
        super().__init__('panoramix_node')
        self._socketio = socketio
        self._namespace = namespace
        self._ansi_escape = re.compile(r'(?:\x1B[@-_]|[\x80-\x9F])[0-?]*[ -/]*[@-~]')
        self._rosout_logs = ''
        for ns in ['', '/asterix', '/obelix', '/assurancetourix']:
            self.create_subscription(Log, f'{ns}/rosout', self.log_callback, 5)
        self.create_subscription(UInt8, '/score', self.score_callback, 5)
        self.ros_thread = Thread(target=self.process, daemon=True)
        self.ros_thread.start()

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
        text = f'[{loglevel}] [{msg.name}] {message}<br>\n'
        self._rosout_logs += text
        self._socketio.emit(
            'console_out',
            {'text': text},
            namespace=self._namespace,
        )
        if loglevel in ('WARN', 'ERROR', 'FATAL'):
            self._socketio.emit(
                'console_msg',
                {'level': loglevel, 'name': msg.name, 'msg': message},
                namespace=self._namespace,
            )

    def score_callback(self, msg):
        """Callback for /score."""
        self._socketio.emit(
            'score',
            {'data': msg.data},
            namespace=self._namespace,
        )

    def process(self):
        """ROS2 Thread."""
        try:
            while True:
                rclpy.spin_once(self, timeout_sec=.0)
                eventlet.greenthread.sleep()
        except ZeroDivisionError:
            return


@app.route('/')
def index():
    return render_template('index.html', async_mode=socketio.async_mode)


def main(args=None):
    rclpy.init(args=args)
    panoramix = Panoramix(socketio, namespace)

    @socketio.on('connect')
    def send_context():
        """Send states and logs."""
        socketio.emit(
            'console_out',
            {'text': panoramix._rosout_logs},
            namespace=namespace,
        )

    socketio.run(
        app,
        host='0.0.0.0',
        port=80,
        debug=False,
    )

    panoramix.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
