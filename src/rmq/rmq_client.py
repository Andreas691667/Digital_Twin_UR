import pika
import threading
import json

# based on https://www.rabbitmq.com/tutorials/tutorial-three-python.html
class Client:
    """Class for the client of the game
    The client is responsible for sending the player's input to the leader using RMQ (pika)
    """

    def __init__(
        self, host="localhost", port=5672
    ) -> None:

        self.__host = host
        self.__port = port
        self.channel = self.create_channel()
        self.channel_type = ""

    def create_channel(self):
        """Create and return a channel object"""
        # The connection object
        connection = pika.BlockingConnection(
            pika.ConnectionParameters(host=self.__host, port=self.__port, credentials=pika.PlainCredentials('Andreas', 'bordtennisbat'))
        )
        # The channel object
        channel = connection.channel()

        return channel

    def configure_incoming_channel(self, on_message_clb, exchange_name, exch_type):
        """Configure the consumer channel"""
        if self.channel_type != "":
            print("Error in RMQ config. Channel already configured")
        else:
            self.channel_type = "incoming"
            self.channel.exchange_declare(
                exchange=exchange_name, exchange_type=exch_type
            )  # for incoming messages

            # Declare the queue (Name is generated uniquely by RMQ)
            # Incoming message queue
            result = self.channel.queue_declare(queue="", exclusive=True)
            incoming_message_queue = result.method.queue

            # Bind the queue to the exchange
            self.channel.queue_bind(
                exchange=exchange_name, queue=incoming_message_queue
            )

            # Create a consumer for the incoming message queue
            self.channel.basic_consume(
                queue=incoming_message_queue,
                on_message_callback=on_message_clb,
                auto_ack=True,
            )

    def start_consumer_thread(self):
        """Start the consumer thread"""
        self.consumer_thread = threading.Thread(target=self.__start_consuming)
        self.consumer_thread.start()

    def configure_outgoing_channel(self, exchange_name, exchange_type):
        """Configure the publisher channel"""
        if self.channel_type != "":
            print("Error in RMQ config. Channel already configured")
        else:
            self.channel_type = "outgoing"
            self.channel.exchange_declare(
                exchange=exchange_name, exchange_type=exchange_type
            )

    def __start_consuming(self):
        """Start consuming messages
        This function runs in a thread"""
        if self.channel_type != "incoming":
            print("Error in RMQ. Channel is not incoming")
        else:
            self.channel.start_consuming()

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
