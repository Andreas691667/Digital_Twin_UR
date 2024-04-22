# from urinterface.robot_connection import RobotConnection
from sys import path

path.append("..")
path.append("../urinterface/src")
from urinterface.robot_connection import RobotConnection
from urinterface.RTDEConnect import RTDEConnect
from config.robot_config import ROBOT_CONFIG
from threading import Thread, Event
from pathlib import Path
from time import sleep

from Controller import Controller
from Monitor import Monitor


class ControllerMonitor:
    """Class responsible for all robot interaction"""

    def __init__(self, task_name, go_to_home=False, file_name_key="") -> None:
        self.conf_file = "record_configuration.xml" # configuration file for RTDE
        self.log_file = (                           # log file for robot output
            file_name_key + "_robot_output.csv"
            if file_name_key != ""
            else f"{task_name}_robot_output.csv"
        )

        self.log_file_path = Path("robot_output") / Path(self.log_file)

        # Robot connection
        # Used for monitoring and dashboard service, e.g. load and play program
        self.robot_connection = RobotConnection(ROBOT_CONFIG.ROBOT_HOST)

        # RTDE
        # Used for writing to the robot registers controller directly
        self.rtde_connection = RTDEConnect(ROBOT_CONFIG.ROBOT_HOST, self.conf_file)

        # shutdown thread
        self.shutdown_event = Event()


        # instantiate controller and monitor
        self.controller = Controller(
            rtde_connection=self.rtde_connection,
            robot_connection=self.robot_connection,
            shutdown_event=self.shutdown_event,
            task_name=task_name,
            go_to_home=go_to_home
        )

        self.monitor = Monitor(
            robot_connection=self.robot_connection,
            conf_file=self.conf_file,
            log_file_path=self.log_file_path,
        )

        # Start threads
        self.wait_for_shutdown()

    def wait_for_shutdown(self):
        """shutdown everything: robot, rmq, threads"""
        while not self.shutdown_event.is_set():
            sleep(0.01)

        print("Shutting down CM...")

        if self.rtde_connection.is_active():
            self.rtde_connection.shutdown()

        self.monitor.stop_monitoring()

        print("CM Shutdown finished successfully!")
        exit(0)
