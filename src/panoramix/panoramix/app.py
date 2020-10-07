#!/usr/bin python3


"""Main Node containing Panoramix managment and monitoring system."""


from flask import Flask, render_template
from flask_socketio import SocketIO

try:
    from panoramix.panoramix import ros_events_thread
except ImportError:
    print('[!] ROS 2 was not found')
    def ros_events_thread(*args, **kwargs): return None


async_mode = None
namespace = '/api'

app = Flask(__name__)
socketio = SocketIO(app, async_mode=async_mode)


@app.route('/')
def index():
    return render_template('index.html', async_mode=socketio.async_mode)


def main(args=None):
    socketio.start_background_task(ros_events_thread, args=args, socketio=socketio, namespace=namespace)
    socketio.run(
        app,
        host='0.0.0.0',
        port=80,
        debug=False,
    )


if __name__ == '__main__':
    main()
