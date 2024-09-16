import socket
import threading
import random

HEADER = 64
PORT = 5050
SERVER = socket.gethostbyname(socket.gethostname())
ADDR = (SERVER, PORT)
FORMAT = 'utf-8'
DISCONNECT_MESSAGE = '!DISCONNECT'

server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
server.bind(ADDR)


def random_generator():
    difference = random.randint(10**4, 10**5)
    L = random.randint(1, 100000 - difference)
    R = L + difference
    X = random.randint(L, R)
    return L, R, X


prisoner_names = ['Prisoner 1', 'Prisoner 2', 'Prisoner 3', 'Prisoner 4']
prisoner_connections = []
escape_order = []

L, R, X = random_generator()


def handle_client(conn, addr, L, R):
    print(f"[NEW CONNECTION] {addr} CONNECTED")
    prisoner_index = len(prisoner_connections)
    prisoner_connections.append(conn)
    conn.send(f"{L} {R}".encode(FORMAT))
    connected = True
    while connected:
        if True: 
            msg = conn.recv(HEADER).decode(FORMAT)
            print(f"[{prisoner_names[prisoner_index]}] {msg}")
            if msg == DISCONNECT_MESSAGE:
                print(f"[{prisoner_names[prisoner_index]}] DISCONNECTED")
                connected = False
            else:
                Y = int(msg)
                if Y > X:
                    message = "The value is too high"
                    conn.send(message.encode(FORMAT))
                    #R = Y
                elif Y < X:
                    message = "The value is too low"
                    conn.send(message.encode(FORMAT))
                    #L = Y
                else:
                    message = "Correct guess"
                    conn.send(message.encode(FORMAT))
                    escape_order.append(conn)
                    print(f"{prisoner_names[prisoner_index]} escaped")
                    break
            #conn.send(f"{L} {R}".encode(FORMAT))

    conn.close()
    print(escape_order)

def start():
    server.listen()
    print(f"[LISTENING] Server is listening on {SERVER}")
    while True:
        conn, addr = server.accept()
        thread = threading.Thread(target=handle_client, args=(conn, addr, L, R))
        thread.start()
        print(f"[ACTIVE CONNECTIONS] {threading.active_count() - 1}")


print('[STARTING] Server is Starting...')
start()
#print(escape_order)

