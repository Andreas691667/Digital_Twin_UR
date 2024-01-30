ROBOT_HOST = "192.168.0.111"
ROBOT_PORT = 30002

import socket
import time

con = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
con.connect((ROBOT_HOST, ROBOT_PORT))


# con.send(b"set_digital_out(2,True)" + b"\n")
while True:
    con.send(b"movej([-1.94, -1.58, 1.16, -1.15, -1.55, 1.18], a=1.0, v=0.6)" + b"\n")
    time.sleep(15)
    con.send(b"movej([1.94, -1.58, 1.16, -1.15, -1.55, 1.18], a=1.0, v=0.6)" + b"\n")
    time.sleep(15)


data = con.recv(1024)
con.close()

print("Received", repr(data))
