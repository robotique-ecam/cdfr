#!/usr/bin/env python3


"""Raspberry pi GPIO emulation through PyQt5 GUI."""


import sys
from threading import Thread

from PyQt5.QtWidgets import QApplication, QPushButton, QWidget


def create_app():
    global w, button
    app = QApplication(sys.argv)
    w = QWidget()
    w.resize(250, 150)
    w.setWindowTitle('Robot Interface')
    button = QPushButton('Enable', w)
    button.setCheckable(True)
    w.show()
    return app.exec_()


class GPIOSim:
    """GPIO emulation."""

    def __init__(self):
        """Init application window."""
        self.IN, self.PUD_DOWN, self.BCM = None, None, None
        self.app_thread = Thread(target=create_app)
        self.app_thread.start()

    def setmode(*args, **kwargs):
        pass

    def setup(*args, **kwargs):
        pass

    def input(self, *args, **kwargs):
        return not button.isChecked()
