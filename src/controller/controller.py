import sys
import socket
import matplotlib.pyplot as plt
import threading
import time

sys.path.append("..")
from rmq.rmq_client import Client
from config.rmq_config import RMQ_CONFIG
from config.robot_config import ROBOT_CONFIG


class Controller:
    """Controller class for the robot"""

    def __init__(self):
        self.robot_con = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.rmq_client = Client(host=RMQ_CONFIG.RMQ_SERVER_IP)
      

    def configure_rmq_client(self):
        """configures rmq client to receive data from DT"""
        self.rmq_client.configure_incoming_channel(
            self.on_rmq_message_cb, RMQ_CONFIG.DT_EXCHANGE, RMQ_CONFIG.FANOUT
        )

        self.rmq_client.start_consumer_thread()

    def robot_connect(self):
        """connects to the robot"""
        self.robot_con.connect((ROBOT_CONFIG.ROBOT_HOST, ROBOT_CONFIG.ROBOT_PORT_IN))

    def robot_disconnect(self):
        """disconnects from the robot"""
        self.robot_con.close()

    def send_ctl_to_robot(self, message):
        """sends control message to the robot"""
        # check type is byte str
        if not isinstance(message, bytes):
            raise ValueError("Message should be of type bytes")
        else:
            self.robot_con.send(message)

    def on_rmq_message_cb(self, ch, method, properties, body):
        """Callback function for when a message is received
        ch: The channel object
        method: The method object
        properties: The properties object
        body: The message body
        This function is called when a message is received from the server
        It is responsible for updating the queue of incoming messages"""
        print(f" [x] Received from DT: {body}")

    
        # WARNING: SEND ONLY MSG TO ROBOT WHEN SURE
        # self.send_ctl_to_robot(body)
