import zmq
import pandas as pd
import subprocess

class Visualiser():

    def __init__(self, port=5556, app_path="") -> None:
        self.port = port
        context = zmq.Context()
        self.socket = context.socket(zmq.PUB)
        self.socket.bind("tcp://*:%s" % port)
        self.application_path = app_path
        self.app_process = None


    def publish_on_topic(self, topic, msg_data):
        # log topic and msg_data to csv file 'zmq_publisher_log.csv'7
        log = pd.DataFrame(data={'topic': [topic], 'msg_data': [msg_data]})
        log.to_csv('zmq_publisher_log.csv', mode='a', header=False, index=False)

        self.socket.send_string(f"{topic} {msg_data}")

    def stop_visualisation(self):
        print("STOPPING ZMQ SOCKET")
        self.socket.send_string(f"stop stop")
        if self.app_process is not None:
            self.app_process.kill()

    def start_application(self):
        self.app_process = subprocess.Popen([self.application_path])
