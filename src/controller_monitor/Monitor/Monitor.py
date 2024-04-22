from sys import path
from threading import Thread
from time import sleep

path.append("..")
from config.msg_config import MSG_TYPES_MONITOR_TO_DT
from config.rmq_config import RMQ_CONFIG
from rmq.RMQClient import Client

class Monitor:
    """Class responsible for monitoring the robot and logging the data to a file"""

    def __init__(self, robot_connection, conf_file, log_file_path):
        self.robot_connection = robot_connection
        self.rmq_client_out_monitor = Client(host=RMQ_CONFIG.RMQ_SERVER_IP)
        self.__configure_rmq_client()

        # Monitor thread
        self.monitor_thread = Thread(
            target=self.robot_connection.start_recording(
                config_file=conf_file,
                filename=log_file_path,
                overwrite=True,
                frequency=20,
                publish_topic=["actual_q"],
                rmq_client=self.rmq_client_out_monitor,
                publish_topic_rmq=MSG_TYPES_MONITOR_TO_DT.MONITOR_DATA,
            )
        )

        self.__start_monitoring()

    def __start_monitoring(self):
        """start the monitor thread"""
        self.monitor_thread.start()

    def __configure_rmq_client(self):
        self.rmq_client_out_monitor.configure_outgoing_channel(
            RMQ_CONFIG.MONITOR_EXCHANGE, RMQ_CONFIG.FANOUT
        )
        print("Monitor RMQ client configured")

    def stop_monitoring(self):
        """stop the monitor thread"""
        print("Shutting down monitor...")
        self.robot_connection.stop_recording()
        sleep(1)
        self.monitor_thread.join()
        print("Monitor shutdown complete")
