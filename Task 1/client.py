import socket
import random
import threading

HEADER = 64
PORT = 5050
FORMAT = 'utf-8'
DISCONNECT_MESSAGE = '!DISCONNECT'
SERVER = socket.gethostbyname(socket.gethostname())
ADDR = (SERVER, PORT)

client = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
client.connect(ADDR)


def send(msg):
    message = str(msg).encode(FORMAT)
    #msg_length = (len(message)) #.encode(FORMAT)
    #msg_length += b' ' * (HEADER - len(msg_length))
    #client.send(msg_length)
    client.send(message)
    #print(client.recv(HEADER).decode(FORMAT))
    
data = client.recv(HEADER).decode(FORMAT)
L, R = map(int, data.split())

while True:
    print("L:", L)
    print("R:", R)
    Y = random.randint(L, R)
    send(str(Y))
    message = client.recv(HEADER).decode(FORMAT)
    print(message)
    if message == "Correct guess":
        print("You Escaped")
        break
    elif message == "The value is too high":
        R = Y
        #print("Enter a higher value: ")
        #guess = input("Enter a higher value: ")
        #send(guess)
    elif message == "The value is too low":
        L = Y
        #print("Enter a lower value: ")
        #guess = input("Enter a lower value: ")
        #send(guess)

send(DISCONNECT_MESSAGE)
client.close()
