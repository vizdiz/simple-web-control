import socket

HOST = "10.29.60.96"
PORT = 8000

with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as s:
    s.connect((HOST, PORT))
    s.sendall(b"5")
    # data = s.recv(1024)

# print(f"Recieved {data!r}")
