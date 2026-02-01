import socket
from collections.abc import Callable

STOP_SIGNAL = '__CLOSE__'
ACKNOWLEDGE_SIGNAL = '__ACK__'
MAX_BYTES_TO_READ = 4096

def handle_data(received_data: str, receiver) -> None:
    """
    Handle data sent from the transmitter
    :param received_data: the data sent by the transmitter.
    :param receiver: a receiver instance.
    """
    receiver.log(f"Received: {received_data}")

    # Send response back
    receiver.send_to_client(f"Echo: {received_data}")

class TCPReceiver:
    def __init__(self, data_handler: Callable = handle_data, log: Callable = print, host='0.0.0.0', port=5000):
        """
        :param data_handler: Callable function that will be passed the received data as bytes, and the receiver object.
        :param log: Callable function for logging; a string will be passed.
        :param host: the host IP to bind to.
        :param port: the port to bind to.
        """
        log('Starting TCP Receiver...')
        self.data_handler = data_handler
        self.log = log
        self.server = socket.socket(socket.AF_INET, socket.SOCK_STREAM) # AF_INET = IPv4, SOCK_STREAM = TCP
        self.server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1) # Allows immediate reuse of the port after the program stops
        self.server.bind((host, port))
        self.server.listen(1) # Backlog queue of 1
        log(f"Server listening on {host}:{port}")
        self.connection, self.client_address = None, None

    def start_listening(self) -> None:
        """
        Wait for a connection from transmitter (blocking); on connect, continually wait for,
        and process, data from transmitter.
        """
        self.connection, self.client_address = self.server.accept()
        self.log(f"Listening to {self.client_address}")
        try:
            self._listen_loop()
        except Exception as e:
            self.log(f"Exception: {e}")
        self.close()

    def _listen_loop(self):
        while True:
            data = self.connection.recv(MAX_BYTES_TO_READ).decode('utf-8')
            if data == STOP_SIGNAL:
                self.log("Stop signal received.")
                self.send_to_client(ACKNOWLEDGE_SIGNAL)
                break

            self.data_handler(data, self)

    def start_listening_forever(self) -> None:
        """
        Continuously wait for connections from transmitters, even after one ends.
        """
        while True:
            self.start_listening()

    def send_to_client(self, data: str) -> bool:
        """
        Send data back to the transmitter.
        :return: True if successful
        """
        if not self.connection: return False
        self.connection.sendall(data.encode('utf-8'))
        return True

    def close(self):
        if self.connection:
            self.connection.close()
        self.connection = None
        self.client_address = None

if __name__ == '__main__':
    receiver = TCPReceiver()
    try:
        receiver.start_listening_forever()
    except KeyboardInterrupt:
        receiver.close()
