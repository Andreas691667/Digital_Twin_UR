import zmq
import subprocess

class Visualiser():
    """Class to manage the visualisation application"""

    def __init__(self, port=5556, app_path="") -> None:
        self.port = port
        context = zmq.Context()
        self.socket = context.socket(zmq.PUB)
        self.socket.bind("tcp://*:%s" % port)
        self.application_path = app_path
        self.app_process = None
        self.topic_names = ["actual_q_0", "actual_q_1", "actual_q_2", "actual_q_3", "actual_q_4", "actual_q_5"]

    def publish_joint_positions(self, joint_positions):
        """Publishes the joint positions on the visualisation application"""
        for i in range(6):
            self.__publish_on_topic(self.topic_names[i], joint_positions[i])

    def __publish_on_topic(self, topic, msg_data):
        """Publishes a message on a topic"""
        self.socket.send_string(f"{topic} {msg_data}")

    def stop_visualisation(self):
        """Stops the visualisation application and the zmq socket"""
        print("Stopping Visualisation")
        self.socket.send_string("stop stop")
        if self.app_process is not None:
            self.app_process.kill()

    def start_application(self):
        """Starts the visualisation application"""
        self.app_process = subprocess.Popen([self.application_path])
