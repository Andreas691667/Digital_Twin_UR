import zmq
import pandas as pd

class ZMQPublisher():

    def __init__(self, port=5556) -> None:
        self.port = port
        context = zmq.Context()
        self.socket = context.socket(zmq.PUB)
        self.socket.bind("tcp://*:%s" % port)

    def publish_on_topic(self, topic, msg_data):
        # # log topic and msg_data to csv file 'zmq_publisher_log.csv'7
        # log = pd.DataFrame(data={'topic': [topic], 'msg_data': [msg_data]})
        # log.to_csv('zmq_publisher_log.csv', mode='a', header=False, index=False)

        self.socket.send_string(f"{topic} {msg_data}")

    def send_stop(self):
        print("STOPPING ZMQ SOCKET")
        self.socket.send_string(f"stop stop")
