ROBOT_HOST = "192.168.0.111"
ROBOT_PORT = 30002

import sys

sys.path.append("..")
import rtde.rtde as rtde
import rtde.rtde_config as rtde_config
import rtde.csv_writer as csv_writer
import rtde.csv_binary_writer as csv_binary_writer


con = rtde.RTDE(ROBOT_HOST, ROBOT_PORT)
con.connect()

con.get_controller_version()

# start data synchronization
if not con.send_start():
    sys.exit()

con.send(b"set_digital_out(2,True)" + b"\n")

# con.send_pause()
con.disconnect()