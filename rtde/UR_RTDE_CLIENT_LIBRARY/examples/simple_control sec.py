ROBOT_HOST = "192.168.0.111"
ROBOT_PORT = 30002

import socket
import time

con = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
con.connect((ROBOT_HOST, ROBOT_PORT))


script_enter_interpreter = b"""
def fun2(): \n
\tinterpreter_mode(clearQueueOnEnter = True, clearOnEnd = True) \n
\tabort() \n
\tend_interpreter() \n
end \n
fun2() \n
"""
script_end_interpreter = b"""
end_interpreter() \n
"""


enter_interprter = b"""interpreter_mode(clearQueueOnEnter = True, clearOnEnd = True) \n"""
abort = b"""abort end_interpreter() \n"""
single_line = b'def myFun(): mystring = "Hello Interpreter" textmsg(mystring) end \n'

con.send(enter_interprter)
con.close()
time.sleep(2)
ROBOT_PORT = 30020
con = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
con.connect((ROBOT_HOST, ROBOT_PORT))

con.send(abort)
data = con.recv(1024)
con.close()

print("Received", repr(data))
