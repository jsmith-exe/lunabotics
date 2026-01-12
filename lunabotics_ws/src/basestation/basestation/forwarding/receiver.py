import socket

class ReceiverNode:
    def __init__(self, host='0.0.0.0', port=5000):
        self.server = socket.socket(socket.AF_INET, socket.SOCK_STREAM) # AF_INET = IPv4, SOCK_STREAM = TCP
        self.server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1) # Allows immediate reuse of the port after the program stops
        self.server.bind((host, port))
        self.server.listen(1) # Backlog queue of 1
        print(f"Server listening on {host}:{port}")
        self.connection, self.client_address = None, None

    def start_listening(self):
        self.connection, self.client_address = self.server.accept()
        print(f"Listening to {self.client_address}")
        while True:
            try:
                data = self.connection.recv(4096) # Bytes to consume
                self.handle_data(data)
            except Exception as e:
                print(f"Exception: {e}")
                self.close()
                return

    def handle_data(self, data):
        data = data.decode('utf-8')
        print(f"Received: {data}")

        # Send response back
        self.send_to_client(f"Echo: {data}")

    def send_to_client(self, data: str):
        if not self.connection: return False
        self.connection.sendall(data.encode('utf-8'))
        return True

    def close(self):
        self.connection.close()
        self.connection = None
        self.client_address = None

if __name__ == '__main__':
    node = ReceiverNode()
    while True:
        try:
            node.start_listening()
        except KeyboardInterrupt:
            node.close()
            break
