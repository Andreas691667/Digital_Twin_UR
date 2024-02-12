import zmq

class ZMQPublisher():

    def __init__(self, port=5556) -> None:
        self.port = port
        context = zmq.Context()
        self.socket = context.socket(zmq.PUB)
        self.socket.bind("tcp://*:%s" % port)

    def publish_on_topic(self, topic, msg_data):
        self.socket.send_string(f"{topic} {msg_data}")


    def send_stop(self):
        print("STOPPING SOCKET")
        self.socket.send_string(f"stop stop")

