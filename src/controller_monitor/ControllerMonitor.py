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
from time import sleep, time
import msvcrt  # for user input

from controller_monitor_states import CM_STATES

from rmq.RMQClient import Client
from config.rmq_config import RMQ_CONFIG
from config.msg_config import MSG_TYPES
from config.task_config import TASK_CONFIG
from ur3e.ur3e import UR3e
import json


class ControllerMonitor:
    """Class responsible for all robot interaction"""

    def __init__(self) -> None:

        # Attributes
        self.STATE = CM_STATES.INITIALIZING  # flag to check if main program is running
        self.block_number = 0  # current block number being processed
        self.task_config = (
            TASK_CONFIG.DYNAMIC_THRESHOLD_SMALL_TASK.copy()
        )  # get own local copy of task config
        self.dt_timer_finished : bool = False # flag to check if overall task is finished
        self.task_finished : bool = False # flag to check if overall task is finished

        self.program_running_name: str = ""
        self.conf_file = "record_configuration.xml"
        self.log_file = "robot_output.csv"
        self.log_file_path = Path("test_results") / Path(self.log_file)
        
        self.task_timer = 0
        # model of the robot
        self.robot_model = UR3e()

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
        # self.publish_task_config_to_dt()

        # Controller thread
        self.controller_thread = Thread(target=self.controller_worker)
        self.controller_thread_event = Event()
        self.controller_queue = Queue()

        # shutdown thread
        self.shutdown_thread = Thread(target=self.shutdown)
        self.shutdown_event = Event()

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

        # # Attributes
        # self.block_number = 0  # current block number being processed
        # self.STATE = CM_STATES.INITIALIZING  # flag to check if main program is running
        # self.task_config = (
        #     TASK_CONFIG.block_config_heart.copy()
        # )  # get own local copy of task config
        # self.dt_timer_finished = False

        # Initialize robot registers
        self.init_robot_registers()

        # Starts threads
        self.monitor_thread.start()
        self.controller_thread.start()
        self.shutdown_thread.start()
        sleep(0.5)
        self.__go_to_home()
        # Display message
        print("\n [USER] Ready to play program. Press '2' to start, 'c' to exit \n")

    def publish_task_config_to_dt(self) -> None:
        """Publish the task to the DT"""

        # construct msg with type: task_config and body: task_config
        msg = f"TASK_CONFIG {self.task_config}"

        self.rmq_client_out.send_message(
            json.dumps(msg), RMQ_CONFIG.DT_EXCHANGE
        )
        print("Task config published to DT")
    
    def publish_IK_solution_to_dt (self, IK_sol) -> None:
        """published IK solution to DT, such that DT can calculate threshold"""
        self.rmq_client_out.send_message(IK_sol, RMQ_CONFIG.DT_EXCHANGE)
        print("IK solution published to DT")

    def recieve_user_input(self) -> None:
        """Blocking call that listens for user input"""
        try:
            k = msvcrt.getwche()
            if k == "c":
                print("Exiting")
                self.shutdown_event.set()
            elif k == "2":
                self.STATE = CM_STATES.NORMAL_OPERATION
            
            elif k == "i" and self.task_finished:
                # invert task
                self.invert_task()
            # reset k
            k = "a"
        except KeyboardInterrupt:
            print("Exiting")
            self.shutdown_event.set()

    def invert_task(self):
        """Invert the task"""
        # invert task
        # set block number to 1
        # set task_finished to False
        # set state to NORMAL_OPERATION
        # initialize task registers

        for i in range(self.task_config[TASK_CONFIG.NO_BLOCKS]):
            self.task_config[i][TASK_CONFIG.ORIGIN], self.task_config[i][TASK_CONFIG.TARGET] = (
            self.task_config[i][TASK_CONFIG.TARGET], self.task_config[i][TASK_CONFIG.ORIGIN])

        self.block_number = 0
        self.task_finished = False
        self.rtde_connection = RTDEConnect(ROBOT_CONFIG.ROBOT_HOST, self.conf_file)

        self.STATE = CM_STATES.NORMAL_OPERATION


    def init_robot_registers(self):
        """initialize the robot
        startbit (bool 65) = False
        detectionbit (bool 66) = False
        activates gripper
        """
        self.load_program("/move-block-init.urp")
        self.play_program()
        sleep(0.01)
        self.load_program("/move_registers.urp")
        print("Robot initialization finished")
        self.STATE = CM_STATES.WAITING_FOR_USER_INPUT

    def __initialize_task_registers(self, values) -> None:
        """initialize the task registers"""
        print(f"Task registers initialized for block: {self.block_number}")
        self.rtde_connection.sendall("in", values)

    def configure_rmq_clients(self):
        """configures rmq client to receive data from DT"""
        self.rmq_client_in.configure_incoming_channel(
            self.on_rmq_message_cb, RMQ_CONFIG.DT_EXCHANGE, RMQ_CONFIG.FANOUT, RMQ_CONFIG.MONITOR_QUEUE
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
            # print(msg_body)
            # print(self.block_number)
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
        print(f"Program '{program_name}' loaded: {succ}")

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

    def __get_register_values (self) -> list:
        """Gets the register values corresponding to the block to move"""
        # Get Origin and Target of current block
        origin = self.task_config[self.block_number][TASK_CONFIG.ORIGIN]
        target = self.task_config[self.block_number][TASK_CONFIG.TARGET]

        # Calculate joint positions
        origin_q_start, origin_q, target_q_start, target_q = self.robot_model.compute_joint_positions_origin_target(
            origin, target
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
        values_vstacked = np.vstack((origin_q_start, origin_q, target_q_start, target_q))
        print(values_vstacked)
        values = np.hstack((origin_q_start, origin_q, target_q_start, target_q))
        values = list(np.array(values).flatten())

        return values

    def controller_worker(self):
        """worker for the controller thread.
        Listens for new control signals from the DT"""
        while not self.controller_thread_event.is_set():

            # Try to get message from RMQ queue, with data from PT
            try:
                if self.rtde_connection.is_active():
                    self.rtde_connection.receive()  # Needed in order to send new data to robot
                msg_type, msg_body = self.controller_queue.get(timeout=0.01)

            # -- NO MESSAGE --
            except Empty:
                # Task has begun
                if self.STATE == CM_STATES.NORMAL_OPERATION:
                    # self.recieve_user_input()
                    # Subtask is done, and there is more blocks to move        

                    if (not self.robot_connection.program_running()) and (
                        self.block_number < self.task_config[TASK_CONFIG.NO_BLOCKS] 
                    ):
                        print(f"Ready to take block number: {self.block_number}")
                        print(f"Time of last task: {time() - self.task_timer if self.task_timer != 0  else -1}")
                        self.task_timer = time()

                        # 1) get register values, by computing inverse kinematics
                        register_values = self.__get_register_values()

                        # 2) initialize task registers
                        self.__initialize_task_registers(register_values)

                        # 3) play the program
                        self.play_program(main_program=True)

                        # 4) Increment block number to next
                        self.block_number += 1

                    # All blocks are done
                    elif self.block_number >= self.task_config[TASK_CONFIG.NO_BLOCKS] and (
                        not self.robot_connection.program_running()                  
                    ):
                        print(f"Time of last task: {time() - self.task_timer if self.task_timer != 0  else -1}")

                        self.__go_to_home()
                        sleep(1)
                        print("\n [USER] Task is done. Press 'i' to perform inverse task. Press 'c' to exit \n")
                        self.rtde_connection.shutdown()
                        self.STATE = CM_STATES.WAITING_FOR_USER_INPUT
                        self.task_finished = True

                elif self.STATE == CM_STATES.WAITING_FOR_USER_INPUT:
                    self.recieve_user_input()

                if self.STATE == CM_STATES.WAITING_FOR_DT:
                    pass

            # -- MESSAGE --
            else:
                # Fault was detected, wait for DT to plan
                if msg_type == MSG_TYPES.WAIT:
                    self.STATE = CM_STATES.WAITING_FOR_DT
                    self.stop_program()

                elif msg_type == MSG_TYPES.TASK_VALIDATED:
                    self.__reconfigure_task(msg_body, decr=False)

                # A resolution was send
                elif msg_type == MSG_TYPES.RESOLVED:
                    new_task = str(msg_body)  # TODO: check if this is necessary
                    self.__reconfigure_task(new_task, decr=True)
                    self.STATE = CM_STATES.NORMAL_OPERATION

                # DT could not resolve
                elif msg_type == MSG_TYPES.COULD_NOT_RESOLVE:
                    print("DT could not resolve")
                    self.robot_connection.popup("DT could not resolve. Task not possible. Exiting")
                    self.shutdown_event.set()

    def __go_to_home(self):
        """Go to home position"""
        home_q = self.robot_model.compute_joint_positions_xy(TASK_CONFIG.HOME_POSITION[TASK_CONFIG.x],
                                                          TASK_CONFIG.HOME_POSITION[TASK_CONFIG.y])
        self.robot_connection.movej(home_q)

    def __reconfigure_task(self, new_task: str, decr:bool) -> None:
        """function for reconfiguring PT task"""
        new_task_dict = ast.literal_eval(new_task)  # convert string to dict
        self.task_config = new_task_dict  # set new task
        
        if decr:
            self.block_number -= 1
        else:
            pass

    def shutdown(self):
        """shutdown everything: robot, rmq, threads"""
        while not self.shutdown_event.is_set():
            sleep(0.01)
        
        print("Shutting down")
        self.stop_program()
        self.controller_thread_event.set()
        self.stop_monitoring()
        self.controller_thread.join()
        self.rmq_client_in.stop_consuming()

        if self.rtde_connection.is_active():
            self.rtde_connection.shutdown()

        print("Shutdown complete")
