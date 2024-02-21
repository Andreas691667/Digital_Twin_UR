# from urinterface.robot_connection import RobotConnection
from sys import path
import numpy as np
import ast

path.append("..")
path.append("../urinterface/src")
from urinterface.robot_connection import RobotConnection
from urinterface.RTDEConnect import RTDEConnect
from config.robot_config import ROBOT_CONFIG
from threading import Thread, Event
from pathlib import Path
from queue import Queue, Empty
from time import sleep

from controller_monitor_states import CM_STATES

from rmq.RMQClient import Client
from config.rmq_config import RMQ_CONFIG
from config.msg_config import MSG_TYPES
from config.task_config import TASK_CONFIG
import json


class ControllerMonitor:
    """Class responsible for all robot interaction"""

    def __init__(self) -> None:
        self.program_running_name: str = ""
        self.conf_file = "record_configuration.xml"
        self.log_file = "robot_output.csv"
        self.log_file_path = Path("test_results") / Path(self.log_file)

        # Robot connection
        # Used for monitoring and dashboard service, e.g. load and play program
        self.robot_connection = RobotConnection(ROBOT_CONFIG.ROBOT_HOST)

        # RTDE
        # Used for writing to the robot controller directly
        self.rtde_connection = RTDEConnect(ROBOT_CONFIG.ROBOT_HOST, self.conf_file)

        # RMQ connection
        self.rmq_client_in = Client(host=RMQ_CONFIG.RMQ_SERVER_IP)
        self.rmq_client_out = Client(host=RMQ_CONFIG.RMQ_SERVER_IP)
        self.configure_rmq_clients()

        # Controller thread
        self.controller_thread = Thread(target=self.controller_worker)
        self.controller_thread_event = Event()
        self.controller_queue = Queue()

        # Monitor thread
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

        # Attributes
        self.block_number = 1  # current block number being processed
        self.STATE = CM_STATES.READY  # flag to check if main program is running
        # get own local copy of task config
        self.task_config = TASK_CONFIG.block_config.copy()
        self.monitor_thread.start()

        self.init_robot_registers()

        self.controller_thread.start()

    def init_robot_registers(self):
        """initialize the robot"""
        self.load_program("/move-block-init.urp")
        self.play_program()
        sleep(0.01)
        print("Robot initialized")
        # self.stop_program()

    def initialize_task_registers(self):
        """initialize the task registers with the waypoints for the current block number"""

        # print(f"Current config with type {type(self.task_config)}: \n {self.task_config}")

        values = self.task_config[self.block_number][TASK_CONFIG.WAYPOINTS]

        # convert to np array and flatten it
        values = np.array(values).flatten()
        # convert to list
        values = list(values)
        print(
            f"Task registers initialized for block: {self.block_number} with values: {values}"
        )

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
        # print(f" [x] Received msg from DT: {body}")
        # get type and body

        try:
            data = json.loads(body)
            msg_type, msg_body = data.split(" ", 1)
            self.controller_queue.put((msg_type, msg_body))
            print("----- DT MESSAGE ----")
            print(msg_type)
            print(msg_body)
            print(self.block_number)
            print("----------------------")
        except ValueError:
            print("Invalid message format")

    def stop_monitoring(self):
        """stop the monitor thread"""
        self.robot_connection.stop_recording()
        sleep(1)
        self.monitor_thread.join()

    def load_program(self, program_name: str) -> None:
        """Load program"""
        succ = self.robot_connection.load_program(program_name)
        self.program_running_name = program_name
        print(f"Program loaded: {succ}")

    def play_program(self, main_program=False) -> None:
        """Start loaded program"""
        program_started = self.robot_connection.play_program()
        if program_started:
            print(f"Program started: {program_started}")
        else:
            pass

        if main_program:
            sleep(1)
            self.STATE = CM_STATES.NORMAL_OPERATION

    def stop_program(self) -> None:
        """stop current exection"""
        self.robot_connection.stop_program()
        print("Program stopped")

    def controller_worker(self):
        """worker for the controller thread.
        Listens for new control signals from the DT"""
        while not self.controller_thread_event.is_set():

            # Try to get message from RMQ queue, with data from PT
            try:
                self.rtde_connection.receive()  # Needed in order to send new data to robot
                msg_type, msg_body = self.controller_queue.get(timeout=0.01)

            # -- NO MESSAGE --
            except Empty:
                # Task has begun
                if self.STATE == CM_STATES.NORMAL_OPERATION:
                    #
                    if (not self.robot_connection.program_running()) and (
                        self.block_number < self.task_config[TASK_CONFIG.NO_BLOCKS]
                    ):
                        self.block_number += 1
                        print(f"Incremented block number to: {self.block_number}")
                        self.initialize_task_registers()
                        self.play_program(main_program=True)
            # -- MESSAGE --
            else:
                # Fault was detected, wait for DT to plan
                if msg_type == MSG_TYPES.WAIT:
                    self.STATE = CM_STATES.WAITING_FOR_DT
                    self.stop_program()

                # A resolution was send
                elif msg_type == MSG_TYPES.RESOLVED:
                    new_task = str(msg_body)  # TODO: check if this is necessary
                    self.__reconfigure_task(new_task)
                    self.STATE = CM_STATES.NORMAL_OPERATION

                # DT could not resolve
                elif msg_type == MSG_TYPES.COULD_NOT_RESOLVE:
                    pass

    def __reconfigure_task(self, new_task: str) -> None:
        """function for reconfiguring PT task"""
        new_task_dict = ast.literal_eval(new_task)        # convert string to dict
        self.task_config = new_task_dict                  # set new task
        self.block_number -= 1                            # reset block_number

    def shutdown(self):
        """shutdown everything: robot, rmq, threads"""
        self.stop_program()
        self.controller_thread_event.set()
        self.stop_monitoring()
        self.controller_thread.join()
        self.rmq_client_in.stop_consuming()
        self.rtde_connection.shutdown()
        print("Shutdown complete")
