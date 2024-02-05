import sys
import socket
sys.path.append("..")
from rmq.rmq_client import Client
from config.rmq_config import RMQ_CONFIG


class Controller:
    def __init__(self):
        self.robot_con = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.rmq_client = Client()

    def configure_rmq_client(self):
        """configures rmq client to receive data from DT"""
        self.rmq_client.configure_incoming_channel(
            self.on_rmq_message_cb, RMQ_CONFIG.DT_EXCHANGE, RMQ_CONFIG.FANOUT
        )

        self.rmq_client.start_consumer_thread()

    def robot_connect(self):
        

    def robot_disconnect(self):
        pass

    def rmq_connect(self):
        pass

    def rmq_disconnect(self):
        pass

    def on_rmq_message_cb(self):
        pass

    