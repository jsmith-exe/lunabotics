import socket
from collections.abc import Callable

def handle_data(received_data: bytes, receiver) -> None:
    """
    Handle data sent from the transmitter
    :param received_data: the data sent by the transmitter.
    :param receiver: a receiver instance.
    """
    received_data = received_data.decode('utf-8')
    print(f"Received: {received_data}")

    # Send response back
    receiver.send_to_client(f"Echo: {received_data}")

class TCPReceiver:
    def __init__(self, data_handler: Callable = handle_data, host='0.0.0.0', port=5000):
        """
        :param data_handler: Callable function that will be passed the received data as bytes, and the receiver object.
        :param host: the host IP to bind to.
        :param port: the port to bind to.
        """
        self.data_handler = data_handler
        self.server = socket.socket(socket.AF_INET, socket.SOCK_STREAM) # AF_INET = IPv4, SOCK_STREAM = TCP
        self.server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1) # Allows immediate reuse of the port after the program stops
        self.server.bind((host, port))
        self.server.listen(1) # Backlog queue of 1
        print(f"Server listening on {host}:{port}")
        self.connection, self.client_address = None, None

    def start_listening(self) -> None:
        """
        Wait for a connection from transmitter (blocking); on connect, continually wait for,
        and process, data from transmitter.
        """
        self.connection, self.client_address = self.server.accept()
        print(f"Listening to {self.client_address}")
        while True:
            try:
                data = self.connection.recv(4096) # Bytes to consume
                self.data_handler(data, self)
            except Exception as e:
                print(f"Exception: {e}")
                self.close()
                return

    def send_to_client(self, data: str) -> bool:
        """
        Send data back to the transmitter.
        :return: True if successful
        """
        if not self.connection: return False
        self.connection.sendall(data.encode('utf-8'))
        return True

    def close(self):
        self.connection.close()
        self.connection = None
        self.client_address = None

if __name__ == '__main__':
    node = TCPReceiver()
    while True:
        try:
            node.start_listening()
        except KeyboardInterrupt:
            node.close()
            break
