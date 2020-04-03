#!/usr/bin python3


"""Main Node containing Panoramix managment and monitoring system."""


from flask import Flask, render_template
from flask_socketio import SocketIO, emit, send


async_mode = None
namespace = "/api"

app = Flask(__name__)
socketio = SocketIO(app, async_mode=async_mode)


def server_events_thread():
    """Wrapper for server side generated events."""
    line = 0
    while True:
        socketio.sleep(5)
        line += 1
        socketio.emit(
            'console_out',
            {'text': f'Console output line {line}'},
            namespace=namespace,
        )


@app.route('/')
def index():
    return render_template('index.html', async_mode=socketio.async_mode)


def main():
    socketio.start_background_task(server_events_thread)
    socketio.run(
        app,
        host='0.0.0.0',
        port=8080,
        debug=True,
    )


if __name__ == '__main__':
    main()
