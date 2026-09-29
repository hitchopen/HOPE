# server.py
import socket

HOST = '0.0.0.0'
PORT = 5001

with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
    s.bind((HOST, PORT))
    s.listen()
    print("Waiting for connection...")
    conn, addr = s.accept()
    print("Connected:", addr)

    while True:
        data = conn.recv(4096)
        if not data:
            break
        print(data.decode())
