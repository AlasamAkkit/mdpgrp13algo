from re import M
import socket
import json


class Client:
    """
    Used as the client for RPI.
    """
    def __init__(self, host, port):
        self.host = host
        self.port = port
        self.socket = socket.socket()

    def connect(self):
        print("=================================Connection=================================")
        print(f"Attempting connection to ALGO at {self.host}:{self.port}")
        self.socket.connect((self.host, self.port))
        print("Connected to ALGO!")

    def close(self):
        print("Closing client socket.")
        self.socket.close()

