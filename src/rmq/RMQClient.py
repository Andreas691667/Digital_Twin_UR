import pika
import threading
import json
import sys

sys.path.append("..")
from config.rmq_credentials import RMQ_CREDENTIALS


# based on https://www.rabbitmq.com/tutorials/tutorial-three-python.html
class Client:
    """Class for the client of the game
    The client is responsible for sending the player's input to the leader using RMQ (pika)
    """

    def __init__(self, host="localhost", port=5672, name="") -> None:

        self.__host = host
        self.__port = port
        self.channels_configured = 0
        self.channels = []
        self.create_channel()
        self.channel = self.channels[0]
        self.channel_type = ""
        self.name = name
        self.consumer_threads = []

    def create_channel(self):
        """Create and return a channel object"""
        # The connection object
        connection = pika.BlockingConnection(
            pika.ConnectionParameters(
                host=self.__host,
                port=self.__port
                # credentials=pika.PlainCredentials(
                #     RMQ_CREDENTIALS.RMQ_USERNAME, RMQ_CREDENTIALS.RMQ_PASSWORD
                # ),
            )
        )
        # The channel object
        channel = connection.channel()
        self.channels.append(channel)

    def configure_incoming_channel(self, on_message_clb, exchange_name, exch_type, queue_name="", channel_no = 0):
        """Configure the consumer channel"""
        if self.channels_configured > 0:
            self.create_channel()

        channel = self.channels[channel_no] 
        self.channel_type = "incoming"
        channel.exchange_declare(
            exchange=exchange_name, exchange_type=exch_type
        )  # for incoming messages

        # Declare the queue (Name is generated uniquely by RMQ)
        # Incoming message queue
        result = channel.queue_declare(queue=queue_name, exclusive=True, durable=True)
        incoming_message_queue = result.method.queue

        # Bind the queue to the exchange
        channel.queue_bind(
            exchange=exchange_name, queue=incoming_message_queue
        )

        # Create a consumer for the incoming message queue
        channel.basic_consume(
            queue=incoming_message_queue,
            on_message_callback=on_message_clb,
            auto_ack=True,
        )

        self.channels_configured += 1

    def start_consumer_thread(self):
        """Start the consumer thread"""
        if self.channel_type != "incoming":
            print("Error in RMQ. Channel is not incoming")
        
        else:
            # create and start threads for all channels
            for i, channel in enumerate(self.channels):
                thread = threading.Thread(target=channel.start_consuming)
                self.consumer_threads.append(thread)
                thread.start()



        # self.RMQ_consumer_thread = threading.Thread(target=self.__start_consuming)
        # self.RMQ_consumer_thread.start()

    def configure_outgoing_channel(self, exchange_name, exchange_type):
        """Configure the publisher channel"""
        if self.channel_type != "":
            print("Error in RMQ outgoing config. Channel already configured")
        else:
            self.channel_type = "outgoing"
            self.channel.exchange_declare(
                exchange=exchange_name, exchange_type=exchange_type
            )

    def __start_consuming(self):
        """Start consuming messages
        This function runs in a thread"""
        if self.channel_type != "incoming":
            print(f"Error in RMQ. Channel is not incoming for channel, name: {self.name}, type: {self.channel_type}")
        else:
            for channel in self.channels:
                channel.start_consuming()

    def stop_consuming(self):
        """Stop consuming messages"""
        if self.channel_type != "incoming":
            print("Error in RMQ. Channel is not incoming")
        else:
            for channel in self.channels:
                channel.stop_consuming()

    def send_message(self, message, exchange_name, r_key=""):
        """Send message to the rabbitmq server
        message: The message to send
        This is done by publishing the message to the outgoing message queue"""
        if self.channel_type != "outgoing":
            pass
            # print(f"Error in RMQ. Channel is not outgoing, {self.channel_type}")
        else:
            self.channel.basic_publish(
                exchange=exchange_name, routing_key=r_key, body=json.dumps(message)
            )
