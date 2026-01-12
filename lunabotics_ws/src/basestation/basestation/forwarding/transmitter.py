import socket
from collections.abc import Callable

class Transmitter:
    def __init__(self, host='localhost', port=5000):
        self.client = socket.socket(socket.AF_INET, socket.SOCK_STREAM) # AF_INET = IPv4, SOCK_STREAM = TCP
        print(f'Connecting to {host}:port')
        self.client.connect((host, port))
        print(f'Connected to {host}:port')

    def send_message(self, message: str, handle_response: Callable | None = None):
        """
        :param message: String message to send.
        :param handle_response: Callable function that will be passed the response as a decoded string.
        :return: the decoded string response.
        """
        self.client.sendall(message.encode('utf-8'))
        response = self.client.recv(4096).decode('utf-8')

        if handle_response:
            handle_response(response)
        return response

    def close(self):
        self.client.close()

if __name__ == '__main__':
    transmitter = Transmitter()
    while True:
        msg = input('> ')
        if not msg: break
        transmitter.send_message(msg)
    transmitter.close()