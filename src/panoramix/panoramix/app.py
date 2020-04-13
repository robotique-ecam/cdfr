#!/usr/bin python3


"""Main Node containing Panoramix managment and monitoring system."""

import re

import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import Log
from flask import Flask, render_template
from flask_socketio import SocketIO, emit, send


async_mode = None
namespace = "/api"

app = Flask(__name__)
socketio = SocketIO(app, async_mode=async_mode)


class Panoramix(Node):
    def __init__(self):
        super().__init__('panoramix_node')
        self._ansi_escape = re.compile(r'(?:\x1B[@-_]|[\x80-\x9F])[0-?]*[ -/]*[@-~]')
        self.subscription = self.create_subscription(Log, '/rosout', self.log_callback, 5)

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
        loglevel = self._loglevel(msg.level)
        message = self._escape_ansi(msg.msg)
        socketio.emit(
            'console_out',
            {'text': f'[{loglevel}] [{msg.name}] {message}<br>'},
            namespace=namespace,
        )
        if loglevel in ('WARN', 'ERROR', 'FATAL'):
            socketio.emit(
                'console_msg',
                {'level': loglevel, 'name': msg.name, 'msg': message},
                namespace=namespace,
            )


def ros_events_thread():
    """Wrapper for server side generated events."""
    panoramix = Panoramix()
    rclpy.spin(panoramix)
    panoramix.destroy_node()
    rclpy.shutdown()



@app.route('/')
def index():
    return render_template('index.html', async_mode=socketio.async_mode)


def main(args=None):
    rclpy.init(args=args)
    socketio.start_background_task(ros_events_thread)
    socketio.run(
        app,
        host='0.0.0.0',
        port=8080,
        debug=True,
    )


if __name__ == '__main__':
    main()
