#!/usr/bin python3


"""Main Node containing Panoramix managment and monitoring system."""


from flask import Flask


app = Flask(__name__)


@app.route('/')
def hello():
    return f'Hello Bot!'


def main():
    app.run(
        host='0.0.0.0',
        port=8080,
    )
