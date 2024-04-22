
from sys import path
import numpy as np
import ast
import msvcrt
from dataclasses import dataclass
from time import sleep
import yaml
import json
from queue import Queue, Empty
from threading import Thread, Event

path.append("..")
from config.grid_config import GRID_CONFIG
from ur3e.ur3e import UR3e
from config.msg_config import (
    MSG_TYPES_DT_TO_CONTROLLER,
    MSG_TYPES_CONTROLLER_TO_DT,
)
from rmq.RMQClient import Client
from config.rmq_config import RMQ_CONFIG



@dataclass
class ControllerStates:
    """Dataclass for controller monitor states"""

    INITIALIZING = 0  # Before establishing connection to broker
    WAITING_FOR_TASK_VALIDATION = 1  # Waiting for task validation
    WAITING_FOR_USER_INPUT = 2  # Waits for user to press enter
    NORMAL_OPERATION = 3  # Performing task
    WAITING_FOR_FAULT_RESOLUTION = 4  # Waiting for resolution
    MANUEL_INTERVENTION = 5  # Waiting for manuel intervention
    SHUTTING_DOWN = 6  # Shutting down


class Controller:
    """Class responsible for controlling the robot"""
    def __init__(self, rtde_connection, robot_connection, shutdown_event, task_name, go_to_home=False) -> None:
        self.STATE = ControllerStates.INITIALIZING  # flag to check if main program is running
        
        self.block_number = 0  # current block number being processed
        self.task_finished: bool = False  # flag to check if overall task is finished
        self.task_validated: bool = False  # flag to check if task is validated
        self.task_config = None             # task configuration
        self.program_running_name: str = ""
        self.cm_shutdown_event = shutdown_event
        # kinematic model of the robot
        self.robot_model = UR3e()

        # setup connections
        self.robot_connection = robot_connection

        # Go to home position if flag is set and exit
        if go_to_home:
            self.__go_to_home()
            exit(0)

        self.rtde_connection = rtde_connection

        # setup RMQ
        self.rmq_client_in = Client(host=RMQ_CONFIG.RMQ_SERVER_IP)
        self.rmq_client_out_controller = Client(host=RMQ_CONFIG.RMQ_SERVER_IP)
        self.__configure_rmq_clients()

        # shutdown thread
        self.shutdown_event = Event()
        self.shutdown_thread = Thread(target=self.shutdown)


        # Controller thread
        self.controller_thread = Thread(target=self.controller_worker)
        self.controller_thread_event = Event()
        self.controller_queue = Queue()

        # load task
        self.load_task(task_name)
        self.shutdown_thread.start()
        self.controller_thread.start()

    def __configure_rmq_clients(self):
        self.rmq_client_in.configure_incoming_channel(
            self.on_rmq_message_cb,
            RMQ_CONFIG.DT_EXCHANGE,
            RMQ_CONFIG.FANOUT,
            RMQ_CONFIG.CONTROLLER_QUEUE,
        )

        self.rmq_client_out_controller.configure_outgoing_channel(
            RMQ_CONFIG.CONTROLLER_EXCHANGE, RMQ_CONFIG.FANOUT
        )

        self.rmq_client_in.start_consumer_thread()
        print("\t [INFO] Controller RMQ clients configured")

    def load_task(self, task_name):
        """Load task from yaml file"""
        try:
            with open(f"../config/tasks/{task_name}.yaml", "r", encoding="UTF-8") as file:
                self.task_config = yaml.safe_load(file)
        except FileNotFoundError:
            print(f"Task {task_name} not found")
            self.shutdown()

    def recieve_user_input(self) -> None:
        """Listen for user input"""
        try:
            k = msvcrt.getwche()
            if k == "c":
                print(f"\t [INFO] User inputted: {k}")
                self.STATE = ControllerStates.SHUTTING_DOWN
                print("State transition -> SHUTTING_DOWN")
            elif k == "2":
                print(f"\t [INFO] User inputted: {k}")
                self.STATE = ControllerStates.NORMAL_OPERATION
                print("\t[STATE] NORMAL_OPERATION")

            elif k == "i" and self.task_finished:
                print(f"\t [INFO] User inputted: {k}")
                self.invert_task()
            # reset k
            k = "a"
        except KeyboardInterrupt:
            print("Exiting")
            self.STATE = ControllerStates.SHUTTING_DOWN
            print("State transition -> SHUTTING_DOWN")

    def invert_task(self):
        """Invert the task"""
        # invert task
        # set block number to 0
        # set task_finished to False
        # set state to NORMAL_OPERATION
        # initialize task registers

        for i in range(self.task_config[GRID_CONFIG.NO_BLOCKS]):
            (
                self.task_config[i][GRID_CONFIG.ORIGIN],
                self.task_config[i][GRID_CONFIG.TARGET],
            ) = (
                self.task_config[i][GRID_CONFIG.TARGET],
                self.task_config[i][GRID_CONFIG.ORIGIN],
            )

        self.block_number = 0
        self.task_finished = False
        # restart rtde connection
        # self.rtde_connection = RTDEConnect(ROBOT_CONFIG.ROBOT_HOST, self.conf_file)
        self.STATE = ControllerStates.INITIALIZING
        print("State transition -> INITIALIZING")

    def __reset_robot_registers(self):
        """initialize the robot
        startbit (bool 65) = False
        detectionbit (bool 66) = False
        activates gripper
        """
        self.load_program("/move-block-init.urp")
        self.play_program()
        sleep(0.01)
        self.load_program("/move_registers.urp")

    def __initialize_task_registers(self, values) -> None:
        """initialize the task registers"""
        print(f"\t [INFO] Task registers initialized for block: {self.block_number}")
        self.rtde_connection.sendall("in", values)

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
            # print(f"\t [DT] {msg_type}")
        except ValueError:
            print("Invalid message format")

    def load_program(self, program_name: str) -> None:
        """Load program"""
        self.robot_connection.load_program(program_name)
        self.program_running_name = program_name

    def play_program(self, main_program=False) -> None:
        """Start loaded program"""
        self.robot_connection.play_program()

        if main_program:
            sleep(1)
            self.STATE = ControllerStates.NORMAL_OPERATION

    def stop_program(self) -> None:
        """stop current exection"""
        self.robot_connection.stop_program()
        print("\t [INFO] Program execution stopped")

    def __get_register_values(self) -> list:
        """Gets the register values corresponding to the block to move"""
        # Get Origin and Target of current block
        origin = self.task_config[self.block_number][GRID_CONFIG.ORIGIN]
        target = self.task_config[self.block_number][GRID_CONFIG.TARGET]

        # Calculate joint positions
        # print(f"task config is: {self.task_config}")
        origin_q_start, origin_q, target_q_start, target_q = (
            self.robot_model.compute_joint_positions_origin_target(origin, target)
        )

        # Check if any is nan
        if np.isnan(origin_q_start).any():
            print("Origin_q_start is nan")
            self.shutdown()
        if np.isnan(origin_q).any():
            print("Origin_q is nan")
            self.shutdown()
        if np.isnan(target_q_start).any():
            print("Target_q_start is nan")
            self.shutdown()
        if np.isnan(target_q).any():
            print("Target_q is nan")
            self.shutdown()

        # Combine and cast to list
        values = np.hstack((origin_q_start, origin_q, target_q_start, target_q))
        values = list(np.array(values).flatten())

        return values

    def publish_task_to_DT(self) -> None:
        """Publish task to DT"""
        msg = f"{MSG_TYPES_CONTROLLER_TO_DT.NEW_TASK} {self.task_config}"
        self.rmq_client_out_controller.send_message(msg, RMQ_CONFIG.CONTROLLER_EXCHANGE)

    def controller_worker(self):
        """worker for the controller thread.
        Listens for new control signals from the DT"""
        while not self.controller_thread_event.is_set():
            # Try to get message from RMQ queue, with data from DT
            try:
                if self.rtde_connection.is_active():
                    self.rtde_connection.receive()  # Needed in order to send new data to robot
                msg_type, msg_body = self.controller_queue.get(timeout=0.01)

            # -- NO MESSAGE --
            except Empty:
                # ---- STATE MACHINE ----
                if self.STATE == ControllerStates.INITIALIZING:
                    self.__initialize_controller()

                elif self.STATE == ControllerStates.WAITING_FOR_TASK_VALIDATION:
                    self.__wait_for_task_validation()

                elif self.STATE == ControllerStates.WAITING_FOR_USER_INPUT:
                    self.recieve_user_input()

                elif self.STATE == ControllerStates.NORMAL_OPERATION:
                    self.__execute_task()

                elif self.STATE == ControllerStates.WAITING_FOR_FAULT_RESOLUTION:
                    pass

                elif self.STATE == ControllerStates.SHUTTING_DOWN:
                    self.shutdown_event.set()

            # -- MESSAGE FROM DT --
            else:
                self.__process_dt_message(msg_type, msg_body)

    # region STATE MACHINE FUNCTIONS
    def __initialize_controller(self):
        """Initialize the controller and set state to waiting for task validation"""
        # Initialize robot registers
        self.__reset_robot_registers()

        # send task to DT for validation
        self.publish_task_to_DT()

        # Set state to waiting for task validation
        self.STATE = ControllerStates.WAITING_FOR_TASK_VALIDATION
        print("State transition -> WAITING_FOR_TASK_VALIDATION")

    def __wait_for_task_validation(self):
        """Wait for task validation from DT"""
        # Wait for DT to validate task
        if self.task_validated:
            self.load_program("/move_registers.urp")
            self.STATE = ControllerStates.WAITING_FOR_USER_INPUT
            print("State transition -> WAITING_FOR_USER_INPUT")
            print(
                "\n \t [USER] Ready to play program. Press '2' to start, 'c' to exit \n"
            )
        else:
            pass
    
    def __execute_task(self):
        """Execute the task"""
        # Subtask is done, and there are more blocks to move
        if (not self.robot_connection.program_running()) and (
            self.block_number < self.task_config[GRID_CONFIG.NO_BLOCKS]
        ):
            print(
                f"\t [INFO] Ready to take block number: {self.block_number} out of {self.task_config[GRID_CONFIG.NO_BLOCKS]-1}"
            )

            # 1) get register values, by computing inverse kinematics
            register_values = self.__get_register_values()

            # 2) initialize task registers
            self.__initialize_task_registers(register_values)

            # ensure that the values are written
            sleep(0.5)

            # 3) play the program
            self.play_program(main_program=True)

            # 4) Increment block number to next
            self.block_number += 1

        # All blocks are done and subtask is finished	
        elif self.block_number == self.task_config[
            GRID_CONFIG.NO_BLOCKS
        ] and (not self.robot_connection.program_running()):
            self.__go_to_home()
            sleep(1)
            print("\t [INFO] Task done")
            self.__reset_robot_registers()
            self.rtde_connection.shutdown()
            self.task_finished = True

            self.STATE = ControllerStates.WAITING_FOR_USER_INPUT
            print("State transition -> WAITING_FOR_USER_INPUT")

            print(
                "\n \t [USER] Press 'i' to perform inverse task. Press 'c' to exit \n"
            )

    # endregion

    def __process_dt_message(self, msg_type: str, msg_body: str) -> None:
        """Process message from DT based on type"""
        
        # Fault was detected, wait for DT to plan
        if msg_type == MSG_TYPES_DT_TO_CONTROLLER.WAIT:
            self.STATE = ControllerStates.WAITING_FOR_FAULT_RESOLUTION
            print("State transition -> WAITING_FOR_FAULT_RESOLUTION")
            self.stop_program()

        # DT has planned a solution
        elif msg_type == MSG_TYPES_DT_TO_CONTROLLER.TASK_VALIDATED:
            # task validated as part of fault resolution
            if self.STATE == ControllerStates.WAITING_FOR_FAULT_RESOLUTION:
                self.__reconfigure_task(msg_body, decr=True)
                self.STATE = ControllerStates.NORMAL_OPERATION
                print("State transition -> NORMAL_OPERATION")
            # task validated as part of initialization
            else:
                self.__reconfigure_task(msg_body, decr=False)
                self.task_validated = True

        # DT could not validate the task
        elif msg_type == MSG_TYPES_DT_TO_CONTROLLER.TASK_NOT_VALIDATED:
            self.robot_connection.popup("Task not validated. Exiting")
            self.STATE = ControllerStates.SHUTTING_DOWN
            print("State transition -> SHUTTING_DOWN")

        # DT could not resolve the fault
        elif msg_type == MSG_TYPES_DT_TO_CONTROLLER.COULD_NOT_RESOLVE:
            print("\t [INFO] DT could not resolve")
            self.robot_connection.popup(
                "DT could not resolve the fault. Task not possible. Exiting"
            )
            self.STATE = ControllerStates.SHUTTING_DOWN
            print("State transition -> SHUTTING_DOWN")

    def __go_to_home(self):
        """Go to home position"""
        home_q = self.robot_model.compute_joint_positions_xy(
            GRID_CONFIG.HOME_POSITION[GRID_CONFIG.x],
            GRID_CONFIG.HOME_POSITION[GRID_CONFIG.y],
        )
        self.robot_connection.movej(home_q)

    def __reconfigure_task(self, new_task: str, decr: bool) -> None:
        """function for reconfiguring PT task"""
        new_task_dict = ast.literal_eval(new_task)  # convert string to dict
        self.task_config = new_task_dict  # set new task

        if decr:
            self.block_number -= 1
        else:
            pass
    
    def shutdown(self):
        """Shut down everything local to controller and set shutdown event from CM"""
        while not self.shutdown_event.is_set():
            sleep(0.01)

        print("Shutting down controller...")
        self.stop_program()
        self.controller_thread_event.set()
        self.controller_thread.join()
        self.rmq_client_in.stop_consuming()
        self.cm_shutdown_event.set()
        print("Controller shutdown complete")
