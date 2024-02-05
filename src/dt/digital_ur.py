import sys

sys.path.append("..")
from rmq.rmq_client import Client
from config.rmq_config import RMQ_CONFIG


import json

class DigitalUR:
    """Class for the digital twin of the UR3e Robot"""

    def __init__(self):
        self.rmq_client_in = Client()
        self.rmq_client_out = Client()

    def configure_rmq_clients(self):
        """configures rmq_client_in to receive data from monitor and rmq_client_out to send data to controller"""
        self.rmq_client_in.configure_incoming_channel(
            self.on_monitor_message, RMQ_CONFIG.MONITOR_EXCHANGE, RMQ_CONFIG.FANOUT
        )

        self.rmq_client_out.configure_outgoing_channel(
            RMQ_CONFIG.DT_EXCHANGE, RMQ_CONFIG.FANOUT
        )
    
    def start_consuming (self) -> None:
        self.rmq_client_in.start_consumer_thread()

    def on_monitor_message(self, ch, method, properties, body) -> None:
        """Callback function for when a message is received
        ch: The channel object
        method: The method object
        properties: The properties object
        body: The message body
        This function is called when a message is received from the monitor """
        # print(f" [x] Received from monitor: {body}")

        data = json.loads(body)
        print(data)
        # x = data["actual_q_0"]

        self.rmq_client_out.send_message(data[20], RMQ_CONFIG.DT_EXCHANGE)
