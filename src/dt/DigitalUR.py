import sys
import json
import threading
from queue import Queue, Empty
import time
import ast
from dataclasses import dataclass
import numpy as np

sys.path.append("..")
import cli_arguments
from rmq.RMQClient import Client

from config.rmq_config import RMQ_CONFIG
from config.msg_config import MSG_TYPES_CONTROLLER_TO_DT, MSG_TYPES_DT_TO_CONTROLLER
from config.grid_config import GRID_CONFIG

from ur3e.ur3e import UR3e

from dt_services.TaskValidator import TaskValidator
from dt_services.TaskTrajectoryEstimator import TaskTrajectoryEstimator
from dt_services.TimingThresholdEstimator import TimingThresholdEstimator
from dt_services.TrajectoryTimingEstimator import TrajectoryTimingEstimator

@dataclass
class MitigationStrategy:
    """Class for the mitigation strategies"""
    SHIFT_ORIGIN = "SHIFT_ORIGIN"
    TRY_PICK_STOCK = "TRY_PICK_STOCK"

@dataclass
class FaultDetectionApproach:
    """Class for the fault detection approaches"""
    MODEL_BASED = "MODEL_BASED"
    TIMING_THRESHOLDS = "TIMING_THRESHOLDS"

@dataclass
class FaultType:
    """Class for the fault types"""
    MISSING_OBJECT = "MISSING_OBJECT"
    PROTECTIVE_STOP = "PROTECTIVE_STOP"
    UNKOWN_FAULT = "UNKOWN_FAULT"
    NO_FAULT = "NO_FAULT"


@dataclass
class DTState:
    """Class for the states of the digital twin"""
    INITIALIZING = 0
    WAITING_TO_RECEIVE_TASK = 1
    VALIDATING_TASK = 2
    WAITING_FOR_TASK_TO_START = 3
    NORMAL_OPERATION = 4
    FAULT_RESOLUTION = 5


class DigitalUR:
    """Class for the digital twin of the UR3e Robot"""

    def __init__(self, mitigation_strategy: str, approach: int, file_name_key: str = ""):
        # state of the digital twin
        self.state: DTState = DTState.INITIALIZING

        # RMQ clients
        self.rmq_client_in = Client(host=RMQ_CONFIG.RMQ_SERVER_IP)
        self.rmq_client_out = Client(host=RMQ_CONFIG.RMQ_SERVER_IP)

        # message queues
        self.controller_msg_queue = Queue()
        self.monitor_msg_queue = Queue()

        # state machine
        self.state_machine_stop_event = threading.Event()
        self.state_machine_thread = threading.Thread(target=self.state_machine)

        # kinematic model of the UR3e robot
        self.robot_model = UR3e()

        # dt services
        self.task_validator = TaskValidator()
        self.timing_estimator = TimingThresholdEstimator(self.robot_model)
        self.trajectory_estimator = TaskTrajectoryEstimator(self.robot_model)
        self.trajectory_timing_estimator = TrajectoryTimingEstimator(self.robot_model)

        self.mitigation_strategy = None
        self.__set_mitigation_strategy(mitigation_strategy)
        self.approach = None
        self.__set_fault_detection_approach(approach)
        self.current_fault: FaultType = None
        self.state_machine_thread.start()

        # parameters for detecting a fault (v.1.1)
        self.time_of_last_message = 0
        self.last_object_detected = False

        self.current_block = -1  # current block number being processed
        self.task_config = None
        self.pick_stock_tried = 1

        # ----- MODEL-BASED FAULT DETECTION -----
        self.timed_task = [] # list of timed tasks, row: [start, target, time]
        self.step_no = 0  # simulation step number
        self.last_pt_time = 0  # last time the PT was monitored
        self.expected_trajectory_q = []
        self.expected_trajectory_time = []
        self.pos_epsilon = 0.02  # allowed error for each joint [rad]
        # log files
        self.traj_file_name = file_name_key + "_dt_trajectory.csv"
        self.error_file_name = file_name_key + "_dt_error_log.csv"

    def __set_fault_detection_approach(self, approach: int) -> None:
        """Set the fault detection approach"""
        if approach == cli_arguments.MODEL_BASED:
            self.approach = FaultDetectionApproach.MODEL_BASED
        elif approach == cli_arguments.TIMING_THRESHOLDS:
            self.approach = FaultDetectionApproach.TIMING_THRESHOLDS

    def __set_mitigation_strategy(self, mitigation_strategy: str) -> None:
        """Set the mitigation strategy"""
        if mitigation_strategy == cli_arguments.SHIFT:
            self.mitigation_strategy = MitigationStrategy.SHIFT_ORIGIN
        elif mitigation_strategy == cli_arguments.STOCK:
            self.mitigation_strategy = MitigationStrategy.TRY_PICK_STOCK

    def configure_rmq_clients(self):
        """Configures rmq_client_in to receive data from monitor
        and rmq_client_out to send data to controller"""

        self.rmq_client_in.configure_incoming_channel(
            self.on_controller_message,
            RMQ_CONFIG.CONTROLLER_EXCHANGE,
            RMQ_CONFIG.FANOUT,
            RMQ_CONFIG.DT_QUEUE_CONTROLLER,
        )
        self.rmq_client_in.configure_incoming_channel(
            self.on_monitor_message,
            RMQ_CONFIG.MONITOR_EXCHANGE,
            RMQ_CONFIG.FANOUT,
            RMQ_CONFIG.DT_QUEUE_MONITOR,
            1,
        )

        self.rmq_client_out.configure_outgoing_channel(
            RMQ_CONFIG.DT_EXCHANGE, RMQ_CONFIG.FANOUT
        )

        print("RMQ clients configured")

    def start_consuming(self) -> None:
        """Start the consumer thread"""
        self.rmq_client_in.start_consumer_thread()

    def on_controller_message(self, ch, method, properties, body) -> None:
        """Callback function for when a message is received
        ch: The channel object
        method: The method object
        properties: The properties object
        body: The message body
        This function is called when a message is received from the controller"""

        try:
            data = json.loads(body)
        except ValueError:
            print("Invalid message received from controller")

        else:
            # print(f"Message received: {data}")
            msg_type, msg_data = data.split(" ", 1)
            self.controller_msg_queue.put((msg_type, msg_data))

    def on_monitor_message(self, ch, method, properties, body) -> None:
        """Callback function for when a message is received
        ch: The channel object
        method: The method object
        properties: The properties object
        body: The message body
        This function is called when a message is received from the monitor"""

        try:
            data_json = json.loads(body)
            msg_type, data = data_json.split(" ", 1)
            data = ast.literal_eval(data)
            data_dict = {
                "timestamp": float(data[0]),
                "actual_q0": float(data[1]),
                "actual_q1": float(data[2]),
                "actual_q2": float(data[3]),
                "actual_q3": float(data[4]),
                "actual_q4": float(data[5]),
                "actual_q5": float(data[6]),
                "actual_qd0": float(data[7]),
                "actual_qd1": float(data[8]),
                "actual_qd2": float(data[9]),
                "actual_qd3": float(data[10]),
                "actual_qd4": float(data[11]),
                "actual_qd5": float(data[12]),
                "safety_status": int(data[13]),
                "output_bit_register_65": bool(data[14]),  # for start bit
                "output_bit_register_66": bool(data[15]),  # for object detection
            }
            self.monitor_msg_queue.put((msg_type, data_dict))
            self.last_pt_time = data_dict["timestamp"]

        except ValueError:
            pass

    def __get_message(self, msg_queue: Queue, block: bool = False):
        """Wait for a message"""
        try:
            type, data = msg_queue.get(block=block)
        except Empty:
            return None, None
        else:
            return type, data
        
    def __update_time_vector(self):
        """updated expected_trajectory_time to have last_pt_time as the first element and accumulating with 0.05"""
        # TODO: Should be done in the trajectory estimator
        # self.expected_trajectory_time = [self.last_pt_time]
        # for i in range(1, len(self.expected_trajectory_q)):
        #     self.expected_trajectory_time.append(self.expected_trajectory_time[i-1] + 0.05)
        # add self.last_pt_time to all elements in expected_trajectory_time
        for i, _ in enumerate(self.expected_trajectory_time):
            self.expected_trajectory_time[i] += self.last_pt_time

    def state_machine(self):
        """The state machine for the digital twin"""
        while not self.state_machine_stop_event.is_set():
            if self.state == DTState.INITIALIZING:
                self.configure_rmq_clients()
                time.sleep(0.5)
                self.start_consuming()
                self.state = DTState.WAITING_TO_RECEIVE_TASK
                print("State transition -> WAITING_TO_RECEIVE_TASK")

            elif self.state == DTState.WAITING_TO_RECEIVE_TASK:
                # wait for new task from controller
                msg_type, msg_data = self.__get_message(self.controller_msg_queue)
                if msg_data:
                    if msg_type == MSG_TYPES_CONTROLLER_TO_DT.NEW_TASK:
                        self.task_config = ast.literal_eval(msg_data)
                        self.state = DTState.VALIDATING_TASK
                        print("State transition -> VALIDATING_TASK")

            elif self.state == DTState.VALIDATING_TASK:
                # validate the task
                valid, validate_msg = self.validate_task()
                # compute timing thresholds
                self.task_config, _, _, _ = self.timing_estimator.compute_thresholds(
                    self.task_config
                )
                self.timed_task = self.trajectory_timing_estimator.get_traj_timings(self.task_config)
                # send validated task to controller
                print(validate_msg)
                self.rmq_client_out.send_message(validate_msg, RMQ_CONFIG.DT_EXCHANGE)

                if valid:
                    # if task is valid, estimate the trajectory,
                    # and go to waiting for task to start state
                    self.expected_trajectory_q, _, _, self.expected_trajectory_time = (
                        self.trajectory_estimator.estimate_trajectory(
                            self.timed_task,
                            start_time=0,
                            save_to_file=True,
                            file_name=self.traj_file_name,
                        )
                    )
                    self.state = DTState.WAITING_FOR_TASK_TO_START
                    self.monitor_msg_queue.queue.clear()  # clear the monitor queue
                    print("State transition -> WAITING_FOR_TASK_TO_START")

            elif self.state == DTState.WAITING_FOR_TASK_TO_START:
                msg_type, msg_data = self.__get_message(self.monitor_msg_queue)
                if msg_data:
                    # check the digital bit 0 of the data
                    # if set go to monitoring state
                    # else pass
                    if self.check_start_bit(msg_data):
                        # update the time vector
                        self.__update_time_vector()
                        print("State transition -> NORMAL_OPERATION")
                        self.state = DTState.NORMAL_OPERATION
                        # Start timer
                        self.time_of_last_message = time.time()

            elif self.state == DTState.NORMAL_OPERATION:
                self.monitor_pt()
            elif self.state == DTState.FAULT_RESOLUTION:
                # Stop program firstly
                self.execute_fault_resolution(f"{MSG_TYPES_DT_TO_CONTROLLER.WAIT} None")
                # resolve the fault and update the task_config
                msg_type = self.plan_fault_resolution()

                # tell task_validator what object is missing
                if self.current_fault == FaultType.MISSING_OBJECT:
                    self.timing_estimator.set_missing_block(self.current_block + 1)

                # Validate task
                self.validate_task()
                fault_msg = f"{msg_type} {self.task_config}"

                # Execute fault resolution
                self.execute_fault_resolution(fault_msg)
                self.state = DTState.WAITING_FOR_TASK_TO_START
                print("State transition -> WAITING_FOR_TASK_TO_START")

    def validate_task(self):
        """Validate the task using the task validator"""
        valid, self.task_config = self.task_validator.validate_task(
            self.task_config.copy()
        )
        if valid:
            msg = f"{MSG_TYPES_DT_TO_CONTROLLER.TASK_VALIDATED} {self.task_config}"
        else:
            msg = f"{MSG_TYPES_DT_TO_CONTROLLER.TASK_NOT_VALIDATED} None"
        return valid, msg

    def plan_fault_resolution(self) -> None:
        """Resolve the current fault"""
        # resolve the fault here based on current fault and mitigation stragety
        # if fault resolved send new data to controller
        # if fault unresovled send could not resolve fault message to controller
        # in both cases go to waiting for task to start state

        if self.current_fault == FaultType.MISSING_OBJECT:
            if self.mitigation_strategy == MitigationStrategy.SHIFT_ORIGIN:
                # print(f"Task config before (1): \n {self.task_config}")
                # # 1) remove the blocks that have already been moved

                # 2) from the current block, change two first rows in its config to the next block
                for block_no in range(  # Iterate over blocks
                    self.current_block + 1,
                    self.task_config[GRID_CONFIG.NO_BLOCKS]
                    - 1,  # ... from the next block to the last block
                ):

                    self.task_config[block_no][GRID_CONFIG.ORIGIN][GRID_CONFIG.x] = (
                        self.task_config[  # Change the x-coordinate to the next block's x-coordinate
                            block_no + 1
                        ][
                            GRID_CONFIG.ORIGIN
                        ][
                            GRID_CONFIG.x
                        ]
                    )
                    self.task_config[block_no][GRID_CONFIG.ORIGIN][GRID_CONFIG.y] = (
                        self.task_config[  # Change the y-coordinate to the next block's y-coordinate
                            block_no + 1
                        ][
                            GRID_CONFIG.ORIGIN
                        ][
                            GRID_CONFIG.y
                        ]
                    )

                    # Change the timing threshold to the next block's threshold
                    self.task_config[block_no][GRID_CONFIG.TIMING_THRESHOLD] = (
                        self.task_config[block_no + 1][GRID_CONFIG.TIMING_THRESHOLD]
                    )

                # remove the last block and decrement the number of blocks
                self.task_config.pop(self.task_config[GRID_CONFIG.NO_BLOCKS] - 1)
                self.task_config[GRID_CONFIG.NO_BLOCKS] -= 1

                # Reset timer
                self.time_of_last_message = time.time()

                # return fault_msg with the new task_config
                return f"{MSG_TYPES_DT_TO_CONTROLLER.RESOLVED}"

            elif self.mitigation_strategy == MitigationStrategy.TRY_PICK_STOCK:
                # For block[j] try PICK_STOCK[i++]
                if self.pick_stock_tried < len(GRID_CONFIG.PICK_STOCK_COORDINATES):
                    self.task_config[self.current_block + 1][GRID_CONFIG.ORIGIN][
                        GRID_CONFIG.x
                    ] = GRID_CONFIG.PICK_STOCK_COORDINATES[self.pick_stock_tried][
                        GRID_CONFIG.ORIGIN
                    ][
                        GRID_CONFIG.x
                    ]
                    self.task_config[self.current_block + 1][GRID_CONFIG.ORIGIN][
                        GRID_CONFIG.y
                    ] = GRID_CONFIG.PICK_STOCK_COORDINATES[self.pick_stock_tried][
                        GRID_CONFIG.ORIGIN
                    ][
                        GRID_CONFIG.y
                    ]
                    self.pick_stock_tried += 1
                    self.time_of_last_message = time.time()  # Reset timer
                    return f"{MSG_TYPES_DT_TO_CONTROLLER.RESOLVED}"
                else:
                    return f"{MSG_TYPES_DT_TO_CONTROLLER.COULD_NOT_RESOLVE}"

        elif self.current_fault == FaultType.PROTECTIVE_STOP:
            pass

        elif self.current_fault == FaultType.UNKOWN_FAULT:
            pass

    def execute_fault_resolution(self, fault_msg) -> None:
        """Execute the fault resolution: send message to controller
        fault_msg: The fault message to send to the controller
        "msg_type, task_config"
        """
        self.rmq_client_out.send_message(fault_msg, RMQ_CONFIG.DT_EXCHANGE)

    # TODO: add proper return type
    def analyse_data(self, data):
        """Check for faults in the data"""
        # if fault present return True, fault_type
        # else return False, FAULT_TYPES.NO_FAULT
        # Meaning of current_block: The block number currently being processed
        # What are we timing?: The time from block k grapped until block k+1 grapped
        # TODO: Check if task is done before doing the rest! I.e. check for more blocks to move here??

        safety_status = data["safety_status"]

        # protective stop
        if safety_status == 3:
            print("Protective stop")
            return True, FaultType.PROTECTIVE_STOP

        # Normal
        elif safety_status == 1:
            if self.approach == FaultDetectionApproach.TIMING_THRESHOLDS:
                return self.analyse_object_detection(data)
            elif self.approach == FaultDetectionApproach.MODEL_BASED:
                return self.analyse_model_divergence(data)

    def analyse_model_divergence(self, data: str):
        """Check if the model diverges from the actual robot"""
        # check if the model diverges from the actual robot
        # if the model diverges from the actual robot, return fault
        # else return no fault
        
        # Get the actual joint positions
        pt_q = [
            data["actual_q0"],
            data["actual_q1"],
            data["actual_q2"],
            data["actual_q3"],
            data["actual_q4"],
            data["actual_q5"]
        ]

        # Get actual time
        pt_time = data["timestamp"]

        # get index in time vector closest to pt_time
        time_idx = np.argmin(np.abs(np.array(self.expected_trajectory_time) - pt_time))

        # get expected joint positions
        expected_q = self.expected_trajectory_q[time_idx]

        # calculate the error
        error = np.abs(np.array(pt_q) - np.array(expected_q))

        # log error to csv
        with open(f"error_logs/{self.error_file_name}", "a") as f:
            f.write(f"{pt_time} {error[0]} {error[1]} {error[2]} {error[3]} {error[4]} {error[5]}\n")

        # if expected_q is last element in expected_trajectory_q, we are done
        # go to waiting to receive task state
        if time_idx == len(self.expected_trajectory_q) - 1:
            print("Task done")
            self.monitor_msg_queue.queue.clear()
            self.state = DTState.WAITING_TO_RECEIVE_TASK
            print("State transition -> WAITING_TO_RECEIVE_TASK")
            return False, FaultType.NO_FAULT

        return False, FaultType.NO_FAULT


    def analyse_object_detection(self, data: str):
        # check if object is detected
        object_detected = data["output_bit_register_66"]

        # If object_detected was the first True in a sequence of booleans, then a new object was grapped
        object_grapped = not self.last_object_detected and object_detected
        self.last_object_detected = object_detected

        # if object detected, reset timer
        # else check if timer has expired. If expired, return fault
        # If we grap an object, we increment the current block being processed, i.e. it is initialized from 0
        if object_grapped:
            self.current_block += 1  # Increment block number
            self.time_of_last_message = time.time()  # Reset timer
            self.pick_stock_tried = 1  # Reset pick_stock_tried
            print(f"Object grapped in block {self.current_block}")

            # If we have grapped the last object, we are done
            # go to waiting to receive task state
            if self.current_block == self.task_config[GRID_CONFIG.NO_BLOCKS] - 1:
                print("Task done")
                # print(f'Task done for timestamp: {data["timestamp"]}')
                self.current_block = -1
                self.monitor_msg_queue.queue.clear()
                self.state = DTState.WAITING_TO_RECEIVE_TASK
                print("State transition -> WAITING_TO_RECEIVE_TASK")

            return (
                False,
                FaultType.NO_FAULT,
            )  # No fault present (TODO: Not needed here?)

        # If we have not grapped an object, we check for timing constraints
        # it is only when the timer have expired that we report a missing object
        else:
            if (
                self.current_block + 1 < self.task_config[GRID_CONFIG.NO_BLOCKS]
            ) and (  # If there are more blocks to move
                time.time()
                - self.time_of_last_message  # ... time passed since last object was grapped
                > self.task_config[self.current_block + 1][
                    GRID_CONFIG.TIMING_THRESHOLD
                ]  # ... the time has expired for next block's threshold
            ):
                print(f"Missing object {self.current_block + 1}")
                return (
                    True,
                    FaultType.MISSING_OBJECT,
                )  # ... a fault present (i.e. missing object)

        return False, FaultType.NO_FAULT


    def check_start_bit(self, data: str) -> bool:
        """Check the digital bit of the data"""
        return data["output_bit_register_65"]

    def monitor_pt(self) -> None:
        """Monitor the PT"""

        _, monitor_data = self.__get_message(self.monitor_msg_queue)

        if monitor_data:
            fault_present, fault_type = self.analyse_data(monitor_data)
            if fault_present:
                print(f"Fault present: {fault_type}")
                self.current_fault = fault_type
                self.state = DTState.FAULT_RESOLUTION
                print("State transition -> FAULT_RESOLUTION")

    def shutdown(self):
        """Shutdown the digital twin"""
        self.state_machine_stop_event.set()
        self.state_machine_thread.join()
        self.rmq_client_in.stop_consuming()
        print("Digital UR shutdown")
