import socket

STOP_SIGNAL = '__CLOSE__'
MAX_BYTES_TO_READ = 4096

class TCPTransmitter:
    def __init__(self, host='localhost', port=5000):
        self.client = socket.socket(socket.AF_INET, socket.SOCK_STREAM) # AF_INET = IPv4, SOCK_STREAM = TCP
        print(f'Connecting to {host}:port')
        self.client.connect((host, port))
        print(f'Connected to {host}:port')

    def send_message(self, message: str, handle_response: bool) -> str | None:
        """
        :param message: String message to send.
        :param handle_response: whether to wait for and return a response; this will be decoded to a string.
        :return: the decoded string response, if a response handler is given.
        """
        self.client.sendall(message.encode('utf-8'))

        if handle_response:
            response = self.client.recv(MAX_BYTES_TO_READ).decode('utf-8')
            return response
        return None

    def close(self):
        self.send_message(STOP_SIGNAL, True) # Wait for acknowledgement to prevent closing before stop signal is sent
        self.client.close()

if __name__ == '__main__':
    transmitter = TCPTransmitter()
    try:
        while True:
            msg = input('> ')
            if not msg: break
            print(transmitter.send_message(msg, False))
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f'Exception: {e}')
    transmitter.close()