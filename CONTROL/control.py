import socket

HOST = input("IP: ")
PORT = int(input("Port: "))

print("Connecting to car...")
with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
    s.connect((HOST, PORT))
    print("Connected!")
    while True:
        message = input("-> ")
        s.send(message.encode())
