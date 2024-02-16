# from urinterface.robot_connection import RobotConnection
from sys import path

path.append("..")
path.append("../urinterface/src")
from urinterface.robot_connection import RobotConnection
from urinterface.RTDEConnect import RTDEConnect
from config.robot_config import ROBOT_CONFIG
from threading import Thread, Event
from pathlib import Path
from queue import Queue, Empty
from time import sleep

from rmq.RMQClient import Client
from config.rmq_config import RMQ_CONFIG
from config.msg_config import MSG_TYPES
from config.task_config import TASK_CONFIG
import json


class ControllerMonitor:
    """Class responsible for all robot interaction"""

    def __init__(self) -> None:
        self.conf_file = "record_configuration.xml"
        self.log_file = "test_motion1.csv"
        self.log_file_path = Path("test_results") / Path(self.log_file)
        self.robot_connection = RobotConnection(ROBOT_CONFIG.ROBOT_HOST)

        self.rtde_connection = RTDEConnect(ROBOT_CONFIG.ROBOT_HOST, self.conf_file)

        self.block_number = 1 # current block number being processed

        self.rmq_client_in = Client(host=RMQ_CONFIG.RMQ_SERVER_IP)
        self.rmq_client_out = Client(host=RMQ_CONFIG.RMQ_SERVER_IP)
        self.configure_rmq_clients()

        self.controller_thread = Thread(target=self.controller_worker)
        self.controller_thread_event = Event()
        self.controller_queue = Queue()

        self.monitor_thread = Thread(
            target=self.robot_connection.start_recording(
                config_file=self.conf_file,
                filename=self.log_file_path,
                overwrite=True,
                frequency=50,
                publish_topic=["actual_q"],
                rmq_client=self.rmq_client_out,
            )
        )

        self.init_robot_registers()

        self.controller_thread.start()

    def init_robot_registers(self):
        """initialize the robot"""
        self.load_program("/move-block-init.urp")
        self.play_program()
        sleep(1)
        print("Robot initialized")
        self.stop_program()

    def initialize_task_registers(self):
        """initialize the task registers with the waypoints for the current block number"""
        values = TASK_CONFIG.block_config[self.block_number][TASK_CONFIG.WAYPOINTS]
        self.rtde_connection.sendall("in", values)

    def configure_rmq_clients(self):
        """configures rmq client to receive data from DT"""
        self.rmq_client_in.configure_incoming_channel(
            self.on_rmq_message_cb, RMQ_CONFIG.DT_EXCHANGE, RMQ_CONFIG.FANOUT
        )

        self.rmq_client_out.configure_outgoing_channel(
            RMQ_CONFIG.MONITOR_EXCHANGE, RMQ_CONFIG.FANOUT
        )

        self.rmq_client_in.start_consumer_thread()
        print("RMQ clients configured")

    def on_rmq_message_cb(self, ch, method, properties, body):
        """Callback function for when a message is received
        ch: The channel object
        method: The method object
        properties: The properties object
        body: The message body
        This function is called when a message is received from the server
        It is responsible for updating the queue of incoming messages"""
        print(f" [x] Received msg from DT: {body}")
        # get type and body

        try:
            data = json.loads(body)
            msg_type, msg_body = data.split(" ", 1)
            self.controller_queue.put((msg_type, msg_body))
        except ValueError:
            print("Invalid message format")

    def start_monitoring(self):
        """start the monitor thread"""
        self.monitor_thread.start()

    def stop_monitoring(self):
        """stop the monitor thread"""
        self.robot_connection.stop_recording()
        sleep(1)
        self.monitor_thread.join()

    def load_program(self, program_name: str) -> None:
        """Load program"""
        succ = self.robot_connection.load_program(program_name)
        print(f"Program loaded: {succ}")

    def play_program(self) -> None:
        """Start loaded program"""
        program_started = self.robot_connection.play_program()
        if program_started:
            print(f"Program started: {program_started}")
        else:
            pass

    def stop_program(self) -> None:
        """stop current exection"""
        self.robot_connection.stop_program()

    def controller_worker(self):
        """worker for the controller thread.
        Listens for new control signals from the DT"""
        while not self.controller_thread_event.is_set():
            try:
                msg_type, msg_body = self.controller_queue.get(timeout=1)
            except Empty:
                pass
            else:
                if msg_type == MSG_TYPES.STOP_PROGRAM:
                    self.stop_program()

    def shutdown(self):
        """shutdown everything: robot, rmq, threads"""
        self.stop_program()
        self.stop_monitoring()
        self.controller_thread_event.set()
        self.controller_thread.join()
        self.rmq_client_in.stop_consuming()
        print("Shutdown complete")
