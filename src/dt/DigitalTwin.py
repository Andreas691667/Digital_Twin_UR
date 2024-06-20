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
from config.timing_config import TIs

from models.robot_model.ur3e import UR3e
from models.timing_model.TimingModel import TimingModel

from dt_modules.FaultResolver import FaultResolver
from dt_modules.TaskValidator import TaskValidator
from dt_modules.TaskTrajectoryEstimator import TaskTrajectoryEstimator
from dt_modules.TimingThresholdEstimator import TimingThresholdEstimator
from dt_modules.TaskTrajectoryTimingEstimator import TaskTrajectoryTimingEstimator
from dt_modules.Visualisation.URVisualiser import Visualiser


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
    PROCESSING_AND_VALIDATING_TASK = 2
    WAITING_FOR_TASK_TO_START = 3
    NORMAL_OPERATION = 4
    FAULT_RESOLUTION = 5


class DigitalTwin:
    """Class for the digital twin of the UR3e Robot
    :param mitigation_strategy: The mitigation strategy to use
    :param approach: The fault detection approach to use
    :param file_name_key: The key for the log files"""

    def __init__(
        self, mitigation_strategy: str, approach: int, file_name_key: str = ""
    ):
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
        self.state_machine_thread = threading.Thread(target=self.__state_machine)

        # models
        self.robot_model = UR3e()
        home_pos = self.robot_model.get_home_ik_solution()
        self.timing_model = TimingModel(home_pos)

        # dt services
        self.fault_resolver = FaultResolver()
        self.task_validator = TaskValidator()
        self.timing_threshold_estimator = TimingThresholdEstimator(self.robot_model)
        self.task_trajectory_estimator = TaskTrajectoryEstimator(self.robot_model)
        self.task_trajectory_timing_threshold_estimator = TaskTrajectoryTimingEstimator(self.robot_model, self.timing_model)
        self.URVisualiser = Visualiser(
            port=5556,
            app_path="dt_modules/Visualisation/Application/DigitalShadowsUR.exe",
        )

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
        self.timed_task = []  # list of timed tasks, row: [start, target, time, description]
        self.last_pt_time = 0  # last time the PT was monitored
        self.last_pt_q = []  # last joint positions of the PT
        self.first_error_time = 0  # time of the first error
        self.last_expected_traj_index = 0  # 'counter' for the expected trajectory
        self.expected_trajectory_q = np.array([])
        self.expected_trajectory_qd = np.array([])
        self.expected_trajectory_time = np.array([])
        self.expected_trajectory_des = np.array([])

        self.pos_epsilon = 0.7  # allowed error for each joint [rad]
        self.time_epsilon = 1  # allowed time for error to sustain [s]

        # log files
        self.traj_file_name = file_name_key + "_dt_trajectory.csv"
        self.error_file_name = file_name_key + "_dt_error_log.csv"
        self.__create_log_files()

    def __create_log_files(self):
        """Create the log files. Overwrite if they already exist"""
        open(f"error_logs/{self.error_file_name}", "w", encoding="UTF-8").close()
        open(f"dt_trajectories/{self.traj_file_name}", "w", encoding="UTF-8").close()

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

    def __configure_rmq_clients(self):
        """Configures rmq_client_in to receive data from monitor
        and rmq_client_out to send data to controller"""

        self.rmq_client_in.configure_incoming_channel(
            self.__on_controller_message,
            RMQ_CONFIG.CONTROLLER_EXCHANGE,
            RMQ_CONFIG.FANOUT,
            RMQ_CONFIG.DT_QUEUE_CONTROLLER,
        )
        self.rmq_client_in.configure_incoming_channel(
            self.__on_monitor_message,
            RMQ_CONFIG.MONITOR_EXCHANGE,
            RMQ_CONFIG.FANOUT,
            RMQ_CONFIG.DT_QUEUE_MONITOR,
            1,
        )

        self.rmq_client_out.configure_outgoing_channel(
            RMQ_CONFIG.DT_EXCHANGE, RMQ_CONFIG.FANOUT
        )

        print("RMQ clients configured")

    def __start_consuming(self) -> None:
        """Start the consumer thread"""
        self.rmq_client_in.start_consumer_thread()

    def __on_controller_message(self, ch, method, properties, body) -> None:
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
            msg_type, msg_data = data.split(" ", 1)
            self.controller_msg_queue.put((msg_type, msg_data))

    def __on_monitor_message(self, ch, method, properties, body) -> None:
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
            self.last_pt_q = [
                data_dict["actual_q0"],
                data_dict["actual_q1"],
                data_dict["actual_q2"],
                data_dict["actual_q3"],
                data_dict["actual_q4"],
                data_dict["actual_q5"],
            ]

        except ValueError:
            pass

    def __get_message(self, msg_queue: Queue, block: bool = False):
        """Wait for a message"""
        try:
            type_, data = msg_queue.get(block=block)
        except Empty:
            return None, None
        else:
            return type_, data

    def __state_machine(self):
        """The state machine for the digital twin"""
        while not self.state_machine_stop_event.is_set():
            if self.state == DTState.INITIALIZING:
                self.__initialize()
            elif self.state == DTState.WAITING_TO_RECEIVE_TASK:
                self.__receive_task()
            elif self.state == DTState.PROCESSING_AND_VALIDATING_TASK:
                self.__process_and_validate_task()
            elif self.state == DTState.WAITING_FOR_TASK_TO_START:
                self.__wait_for_task_to_start()
            elif self.state == DTState.NORMAL_OPERATION:
                self.__monitor_pt()
            elif self.state == DTState.FAULT_RESOLUTION:
                self.__resolve_fault()

    # region ---- STATE MACHINE FUNCTIONS ----
    def __wait_for_task_to_start(self):
        _, msg_data = self.__get_message(self.monitor_msg_queue)
        if msg_data:
            # check the digital bit 0 of the data
            # if set go to monitoring state
            if self.__check_start_bit(msg_data):
                # update the time vector to align with pt's time
                self.expected_trajectory_time = (
                    self.task_trajectory_estimator.update_time_vector(
                        self.last_pt_time,
                        self.expected_trajectory_time,
                        self.last_expected_traj_index,
                    )
                )
                self.state = DTState.NORMAL_OPERATION
                print("State transition -> NORMAL_OPERATION")
                # Start timer
                self.time_of_last_message = time.time()

    def __receive_task(self):
        """Try to receive a new task from the controller"""
        msg_type, msg_data = self.__get_message(self.controller_msg_queue)
        if msg_data:
            if msg_type == MSG_TYPES_CONTROLLER_TO_DT.NEW_TASK:
                self.task_config = ast.literal_eval(msg_data)
                self.state = DTState.PROCESSING_AND_VALIDATING_TASK
                print("State transition -> PROCESSING_AND_VALIDATING_TASK")

    def __initialize(self):
        """Initialize the digital twin"""
        self.__configure_rmq_clients()
        time.sleep(0.5)  # wait for the RMQ clients to be configured
        self.__start_consuming()
        if self.approach == FaultDetectionApproach.MODEL_BASED:
            self.__start_visualisation()
        self.state = DTState.WAITING_TO_RECEIVE_TASK
        print("State transition -> WAITING_TO_RECEIVE_TASK")

    def __resolve_fault(self):
        """Try to resolve the current fault"""
        # tell controller to wait
        self.__send_message_to_controller(f"{MSG_TYPES_DT_TO_CONTROLLER.WAIT} None")

        # resolve the fault and update the task_config
        fault_resolved = self.__plan_fault_resolution()

        # tell task_validator what object is missing if that is the case
        if self.current_fault == FaultType.MISSING_OBJECT:
            self.timing_threshold_estimator.set_missing_block(self.current_block + 1)

        # if fault resolved, go to validating task state
        if fault_resolved:
            self.first_error_time = 0
            self.state = DTState.PROCESSING_AND_VALIDATING_TASK
            print("State transition -> PROCESSING_AND_VALIDATING_TASK")

        # if fault not resolved, go to waiting to receive task state
        else:
            self.__send_message_to_controller(
                f'{MSG_TYPES_DT_TO_CONTROLLER.COULD_NOT_RESOLVE} ""'
            )
            self.state = DTState.WAITING_TO_RECEIVE_TASK
            print("State transition -> WAITING_TO_RECEIVE_TASK")

    def __process_and_validate_task(self):
        """Process and validate the task"""
        # validate the task (task_config is updated also)
        valid, validate_msg = self.__validate_task()

        # if task is valid, process it and estimate the trajectory
        # go to waiting for task to start state
        if valid:
            self.__process_task()  # process the task
            self.__estimate_trajectory()  # estimate the trajectory
            self.monitor_msg_queue.queue.clear()  # clear the monitor queue
            self.current_fault = None  # reset fault
            self.state = DTState.WAITING_FOR_TASK_TO_START
            print("State transition -> WAITING_FOR_TASK_TO_START")

        # if task is not valid, go to waiting to receive task state
        else:
            self.state = DTState.WAITING_TO_RECEIVE_TASK
            print("State transition -> WAITING_TO_RECEIVE_TASK")

        # send validated task (not containing thresholds) to controller
        self.rmq_client_out.send_message(validate_msg, RMQ_CONFIG.DT_EXCHANGE)

    # endregion

    def __start_visualisation(self):
        """Start the Visualisation application"""
        self.URVisualiser.start_application()

    def __estimate_trajectory(self):
        """Estimate the trajectory based on the timed task"""
        if self.current_fault:
            # find first position in timed_task that is not equal to [None]*6
            first_pos = None
            for _, elem in enumerate(self.timed_task):
                if not np.array_equal(elem[0:6], [None] * 6):
                    first_pos = elem[0:6]
                    break

            # get duration between last_pt_q and first_pos
            duration = self.timing_model.get_duration_between_positions(
                np.vstack((self.last_pt_q, first_pos))
            )
            # update timed_task with new task segment, TODO: [-1] is a placeholder for the TI.Type
            self.timed_task = np.vstack(
                (
                    np.concatenate((self.last_pt_q, first_pos, duration, [-1])),
                    self.timed_task,
                )
            )

        # if task is valid, estimate the trajectory for the timed task,
        # and go to waiting for task to start state
        expected_q, expected_qd, _, expected_t, expected_des = (
            self.task_trajectory_estimator.estimate_trajectory(
                self.timed_task,
                start_time=0,
                save_to_file=False,
            )
        )

        # if expected_trajectory_q is empty, set it to expected_q
        if self.expected_trajectory_q.size == 0:
            self.expected_trajectory_q = expected_q
            self.expected_trajectory_qd = expected_qd
            self.expected_trajectory_time = expected_t
            self.expected_trajectory_des = expected_des

        # else append expected_q to expected_trajectory_q
        # discard the everything after last_expected_traj_index
        else:
            self.expected_trajectory_q = np.append(
                self.expected_trajectory_q[0 : self.last_expected_traj_index],
                expected_q,
                axis=0,
            )
            
            self.expected_trajectory_qd = np.append(
                self.expected_trajectory_qd[0 : self.last_expected_traj_index],
                expected_qd,
                axis=0,
            )

            self.expected_trajectory_time = np.append(
                self.expected_trajectory_time[0 : self.last_expected_traj_index],
                expected_t,
                axis=0,
            )
            self.expected_trajectory_des = np.append(
                self.expected_trajectory_des[0 : self.last_expected_traj_index],
                expected_des,
                axis=0,
            )

    def __validate_task(self):
        """Validate the task using the task validator"""
        valid, self.task_config = self.task_validator.validate_task(
            self.task_config.copy(),
            start_block=self.current_block + 1
        )
        if valid:
            msg = f"{MSG_TYPES_DT_TO_CONTROLLER.TASK_VALIDATED} {self.task_config}"
        else:
            msg = f"{MSG_TYPES_DT_TO_CONTROLLER.TASK_NOT_VALIDATED} None"
        return valid, msg

    def __process_task(self):
        """Process the task depending on the fault detection approach"""
        # compute timing thresholds if using timing thresholds approach
        if self.approach == FaultDetectionApproach.TIMING_THRESHOLDS:
            self.task_config, _, _, _ = self.timing_threshold_estimator.compute_thresholds(
                self.task_config
            )

        # compute task trajectory timings if using model-based approach
        elif self.approach == FaultDetectionApproach.MODEL_BASED:
            init = True if not self.current_fault else False
            if self.current_block != self.task_config[GRID_CONFIG.NO_BLOCKS] - 1:
                self.timed_task = (
                    self.task_trajectory_timing_threshold_estimator.get_task_trajectory_timings(
                        self.task_config,
                        start_block=self.current_block + 1,
                        initializing=init,
                    )
                )

    def __plan_fault_resolution(self) -> None:
        """Resolve the current fault using the FaultResolver"""
        # resolve the fault here based on current fault and mitigation stragety
        # if fault resolved send new data to controller
        # if fault unresovled send could not resolve fault message to controller
        # in both cases go to waiting for task to start state

        if self.current_fault == FaultType.MISSING_OBJECT:
            # only if there are still blocks
            if self.current_block + 1 < self.task_config[GRID_CONFIG.NO_BLOCKS]:
                if self.mitigation_strategy == MitigationStrategy.SHIFT_ORIGIN:
                    shift_thresholds = (
                        True
                        if self.approach == FaultDetectionApproach.TIMING_THRESHOLDS
                        else False
                    )
                    fault_resolved, self.task_config = (
                        self.fault_resolver.shift_origins(
                            self.task_config, self.current_block, shift_thresholds
                        )
                    )
                    self.time_of_last_message = time.time()  # Reset timer
                    return fault_resolved

                elif self.mitigation_strategy == MitigationStrategy.TRY_PICK_STOCK:
                    fault_resolved, self.task_config, self.pick_stock_tried = (
                        self.fault_resolver.use_stock(
                            self.task_config, self.current_block, self.pick_stock_tried
                        )
                    )
                    self.time_of_last_message = time.time()  # Reset timer
                    return fault_resolved

        elif self.current_fault == FaultType.PROTECTIVE_STOP:
            pass

        elif self.current_fault == FaultType.UNKOWN_FAULT:
            pass

    def __send_message_to_controller(self, msg) -> None:
        """Send a message to the controller
        fault_msg: The message to send to the controller
        FORMAT: 'MSG_TYPE DATA'"""
        self.rmq_client_out.send_message(msg, RMQ_CONFIG.DT_EXCHANGE)

    def __reset_task(self):
        """Resets the task, to be called when task is done"""
        print("Task done")
        self.current_block = -1
        self.monitor_msg_queue.queue.clear()
        self.last_pt_q = np.array([])
        self.state = DTState.WAITING_TO_RECEIVE_TASK
        print("State transition -> WAITING_TO_RECEIVE_TASK")
        
    def __analyse_data(self, data):
        """Check for faults in the data"""
        # if fault present return True, fault_type
        # else return False, FAULT_TYPES.NO_FAULT
        # Meaning of current_block: The block number currently being processed
        # What are we timing?: The time from block k grapped until block k+1 grapped
        # TODO: Check if task is done before doing the rest! I.e. check for more blocks to move here??

        safety_status = data["safety_status"]

        # protective stop
        if safety_status == 3:
            print("!!Protective Stop encountered!!")
            return True, FaultType.PROTECTIVE_STOP

        # Normal
        elif safety_status == 1:
            if self.approach == FaultDetectionApproach.TIMING_THRESHOLDS:
                return self.__analyse_timing_thresholds(data)
            elif self.approach == FaultDetectionApproach.MODEL_BASED:
                return self.__analyse_model_divergence(data)

    def __analyse_model_divergence(self, data: str):
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
            data["actual_q5"],
        ]

        # Get actual time
        pt_time = data["timestamp"]

        # get index in time vector closest to pt_time
        time_idx = np.argmin(np.abs(np.array(self.expected_trajectory_time) - pt_time))

        # get expected joint positions
        expected_q = self.expected_trajectory_q[time_idx]
        expected_q_des = self.expected_trajectory_des[time_idx]

        # publish the dt joint positions to the visualiser
        self.URVisualiser.publish_joint_positions(expected_q)

        # calculate the error
        error = np.abs(np.array(pt_q) - np.array(expected_q))

        # check if error is above epsilon for each joint
        faults = [False, False, False, False, False, False]
        for i in range(6):
            if error[i] > self.pos_epsilon:
                faults[i] = True
                self.first_error_time = (
                    pt_time if self.first_error_time == 0 else self.first_error_time
                )
        if not any(faults):
            self.first_error_time = 0
            

        # log error to csv
        with open(f"error_logs/{self.error_file_name}", "a", encoding="UTF-8") as f:
            f.write(
                f"{pt_time} {error[0]} {error[1]} {error[2]} {error[3]} {error[4]} {error[5]} {faults[0]} {faults[1]} {faults[2]} {faults[3]} {faults[4]} {faults[5]}\n"
            )

        # if expected_q is last element in expected_trajectory_q, we are done
        # go to waiting to receive task state
        if time_idx == len(self.expected_trajectory_q) - 1:
            self.__reset_task()
            return False, FaultType.NO_FAULT

        # Check if object was gripped, increment current_block
        self.__analyse_object_grip(data)

        self.last_pt_q = pt_q
        self.last_expected_traj_index = time_idx

        # if any faults are present and the time since the first error is above epsilon, there is a fault
        fault_duration = pt_time - self.first_error_time
        if any(faults) and fault_duration >= self.time_epsilon:
            print(
                f"\t [FAULT] Model diverged from PT for {round(fault_duration, 2)} seconds"
            )
            if expected_q_des == TIs.TYPES.MOVE_WITH_OBJECT:
                return True, FaultType.MISSING_OBJECT
            else:
                return True, FaultType.UNKOWN_FAULT
        else:
            return False, FaultType.NO_FAULT

    def __analyse_object_grip(self, data: str):
        """Check if an object is gripped"""
        # check if object is detected
        object_detected = data["output_bit_register_66"]

        # If object_detected was the first True in a sequence of booleans, then a new object was grapped
        object_grapped = not self.last_object_detected and object_detected
        self.last_object_detected = object_detected

        # if object detected, reset timer
        # else check if timer has expired. If expired, return fault
        # If we grap an object, we increment the current block being processed, i.e. it is initialized from -1
        if object_grapped:
            self.current_block += 1  # Increment block number
            self.pick_stock_tried = 1  # Reset pick_stock_tried
            print(f"Object grapped in block {self.current_block}")

        return object_grapped

    def __analyse_timing_thresholds(self, data: str):
        """Check if the timing thresholds are met"""
        # check if object is gripped
        if self.__analyse_object_grip(data):
            self.time_of_last_message = time.time()  # Reset timer
            # If we have grapped the last object, we are done
            # go to waiting to receive task state
            if self.current_block == self.task_config[GRID_CONFIG.NO_BLOCKS] - 1:
                self.__reset_task()

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

    def __check_start_bit(self, data: str) -> bool:
        """Check the digital bit of the data"""
        return data["output_bit_register_65"]

    def __monitor_pt(self) -> None:
        """Monitor the PT"""

        _, monitor_data = self.__get_message(self.monitor_msg_queue)

        if monitor_data:
            fault_present, fault_type = self.__analyse_data(monitor_data)
            if fault_present:
                print(f"Analysis result: {fault_type}")
                self.current_fault = fault_type
                self.state = DTState.FAULT_RESOLUTION
                print("State transition -> FAULT_RESOLUTION")

    def shutdown(self):
        """Shutdown the digital twin"""
        # save
        if self.approach == FaultDetectionApproach.MODEL_BASED:
            self.URVisualiser.stop_visualisation()
            if self.expected_trajectory_q.size > 0:
                self.task_trajectory_estimator.save_traj_to_file(
                    self.traj_file_name,
                    trajectory_q=self.expected_trajectory_q,
                    trajectory_qd=self.expected_trajectory_qd,
                    time=self.expected_trajectory_time,
                )

        self.state_machine_stop_event.set()
        self.state_machine_thread.join()
        self.rmq_client_in.stop_consuming()
        print("Digital UR shutdown finished successfully!")
