import socket
import json
import time


class Server:
    def __init__(self, host, port):
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.host = host
        self.port = port

    def start(self):
        print(f"starting server at {self.host}:{self.port}")
        self.socket.bind((self.host, self.port))
        print(f"Socket binded to port {self.port}")
        self.socket.listen()
        print("Listening for connection...")
        self.clientSocket, self.address = self.socket.accept()
        print('Got connection from', self.address)

    def send(self, d):
        data = d.encode()
        self.clientSocket.send(data)

    def receive(self):
        msg = self.clientSocket.recv(1024)
        data = msg.decode()
        return(data)

    def close(self):
        print("Closing server socket.")
        self.socket.close()
