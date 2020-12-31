#!/usr/bin/env python3


"""Exceptions."""


class MatchStartedException(Exception):
    def __init__(self, message, errors):
        super().__init__(message)
        self.errors = "Match has already been started"
