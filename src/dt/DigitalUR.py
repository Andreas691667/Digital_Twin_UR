import sys
import json
import threading
from queue import Queue, Empty
import time
import numpy as np

sys.path.append("..")
from rmq.RMQClient import Client
from config.rmq_config import RMQ_CONFIG
from digitalur_states import DT_STATES
from digitalur_fault_types import FAULT_TYPES
from config.msg_config import MSG_TYPES, MSG_TOPICS
from config.task_config import TASK_CONFIG
from ur3e.ur3e import UR3e
from task_validator.TaskValidator import TaskValidator


class DigitalUR:
    """Class for the digital twin of the UR3e Robot"""

    def __init__(self):
        self.state: DT_STATES = DT_STATES.INITIALIZING
        self.rmq_client_in = Client(host=RMQ_CONFIG.RMQ_SERVER_IP)
        self.rmq_client_out = Client(host=RMQ_CONFIG.RMQ_SERVER_IP)
        self.msg_queue = Queue()
        self.state_machine_stop_event = threading.Event()
        self.state_machine_thread = threading.Thread(target=self.state_machine)

        # model of the UR3e robot
        self.robot_model = UR3e()

        self.current_fault: FAULT_TYPES = None
        self.state_machine_thread.start()

        # parameters for detecting a fault
        self.time_of_last_message = 0
        self.last_object_detected = False

        self.current_block = -1  # current block number being processed
        self.task_config = TASK_CONFIG.DYNAMIC_THRESHOLD_TWO_BLOCKS.copy()
        self.pick_stock_tried = 1

        self.task_validator = TaskValidator()

    def configure_rmq_clients(self):
        """Configures rmq_client_in to receive data from monitor
        and rmq_client_out to send data to controller"""
        self.rmq_client_in.configure_incoming_channel(
            self.on_monitor_message, RMQ_CONFIG.MONITOR_EXCHANGE, RMQ_CONFIG.FANOUT, RMQ_CONFIG.DT_QUEUE
        )

        self.rmq_client_out.configure_outgoing_channel(
            RMQ_CONFIG.DT_EXCHANGE, RMQ_CONFIG.FANOUT
        )

        print("RMQ clients configured")

    def start_consuming(self) -> None:
        """Start the consumer thread"""
        self.rmq_client_in.start_consumer_thread()

    def on_monitor_message(self, ch, method, properties, body) -> None:
        """Callback function for when a message is received
        ch: The channel object
        method: The method object
        properties: The properties object
        body: The message body
        This function is called when a message is received from the monitor"""

        # format data as string
        data = json.loads(body)
        data_dict = {
            "timestamp": float(data[0]),
            "actual_q0": float(data[1]),
            "actual_q2": float(data[2]),
            "actual_q3": float(data[3]),
            "actual_q4": float(data[4]),
            "actual_q5": float(data[5]),
            "actual_q6": float(data[6]),
            "output_bit_register_65": bool(data[7]),  # for start bit
            "output_bit_register_66": bool(data[8]),  # for object detection
        }
        # data_dict = {
        #     MSG_TOPICS.TIMESTAMP[0]: MSG_TOPICS.TIMESTAMP[1],
        #     MSG_TOPICS.START_BIT[0]: MSG_TOPICS.START_BIT[1],
        #     MSG_TOPICS.OBJECT_DETECTED[0]: MSG_TOPICS.OBJECT_DETECTED[1],
        # }

        # put data in the queue
        self.msg_queue.put(data_dict)

    def state_machine(self):
        """The state machine for the digital twin"""
        while not self.state_machine_stop_event.is_set():
            if self.state == DT_STATES.INITIALIZING:
                self.configure_rmq_clients()
                self.start_consuming()

                # validate the task
                self.validate_task()


                self.state = DT_STATES.WAITING_FOR_TASK_TO_START
                print("State transition -> WAITING_FOR_TASK_TO_START")
                
            elif self.state == DT_STATES.WAITING_FOR_TASK_TO_START:
                try:
                    data = self.msg_queue.get()
                except Empty:
                    pass
                else:
                    # check the digital bit 0 of the data
                    # if set go to monitoring state
                    # else pass
                    if self.check_start_bit(data):
                        print("State transition -> MONITORING_PT")
                        self.state = DT_STATES.MONITORING_PT
                        # Start timer
                        self.time_of_last_message = time.time()

                    else:
                        pass
            elif self.state == DT_STATES.MONITORING_PT:
                self.monitor_pt()
            elif self.state == DT_STATES.FAULT_RESOLUTION:
                # stop program firstly
                self.execute_fault_resolution(f"{MSG_TYPES.WAIT} None")
                fault_msg = self.plan_fault_resolution(TASK_CONFIG.MITIGATION_STRATEGIES.SHIFT_ORIGIN)
                self.validate_task()
                self.execute_fault_resolution(fault_msg)
                self.state = DT_STATES.WAITING_FOR_TASK_TO_START
                print("State transition -> WAITING_FOR_TASK_TO_START")
    
    def __compute_ik_solutions (task_config, robot_model) -> np.ndarray:
        """Computes the inverse kinematics solutions
            returns 3D matrix:
                row: block number
                column: the four positions for a task
                depth: solutions for each position
        """
        number_of_blocks: int = task_config[TASK_CONFIG.NO_BLOCKS] 
        solutions: np.ndarray = np.zeros(shape=(number_of_blocks, 4, 6))
        
        # For every block (4 coordinates) calculate IK (for 4 coordinates)
        for bn in range(number_of_blocks):
            origin = task_config[bn][TASK_CONFIG.ORIGIN]
            target = task_config[bn][TASK_CONFIG.TARGET]
            origin_q_start, origin_q, target_q_start, target_q = robot_model.compute_joint_positions_origin_target(
                origin, target
            )
            
            # set solutions
            solutions[bn, 0, :] = origin_q_start
            solutions[bn, 1, :] = origin_q
            solutions[bn, 2, :] = target_q_start
            solutions[bn, 3, :] = target_q

        return solutions

    def __get_distances_of_leading_axis(self, joint_positions: np.ndarray) -> np.ndarray:
        """
        Get sliding windows differences
        If input is NxM, then output is (N-1)xM
        """
        # sliding window differences
        joint_positions_differences = np.abs(joint_positions[:-1, :] - joint_positions[1:, :])
        leading_axis = np.argmax(joint_positions_differences, axis=1)
        differences_leading_axis = joint_positions_differences[np.arange(len(joint_positions_differences)), leading_axis]
        return differences_leading_axis

    def __get_durations_of_leading_axis(self, distances_leading_axis: np.ndarray) -> np.ndarray:
        """
        Calculates the durations of the leading axis's
        Input: Distances of the leading axes between each position
        Output: Time it takes to traverse those distances
        Note: This may be calculated differently for shorter movement, i.e. if it does not reach max speed
        """
        JOINT_SPEED_MAX_DEG = 60 # deg/s
        JOINT_ACCELERATION_DEG = 80 # deg/s^2
        GRAP_TIME = 0.7 # s
        JOINT_SPEED_MAX_RAD = np.deg2rad(JOINT_SPEED_MAX_DEG) # rad/s
        JOINT_ACCELERATION_RAD = np.deg2rad(JOINT_ACCELERATION_DEG) # rad/s^2
        ACCELERATION_TIME = JOINT_SPEED_MAX_RAD / JOINT_ACCELERATION_RAD # Time it takes to reach maximun velocity
        ACCELERATION_DIST = 1/2 * JOINT_ACCELERATION_RAD * ACCELERATION_TIME**2
        
        # For every distance the robot have to travel it does:
        # 1) Move with constant acceleration
        # 2) Move with with constant max speed
        # 3) Move with constant decceleration
        # If the movement is short in distance, it might skip step 2.

        all_durations = []
        for _, distance in enumerate(distances_leading_axis):
            # Check if movement reaches max speed
            # Does not reach max speed
            if ACCELERATION_DIST >= distance/2: # If the acceleration distance is greater than half of the distance, then it does not reach max speed
                # Calculate time to reach half-distance
                # d = 1/2 * a * t^2 => t = sqrt{2d/a}
                duration_half_distance = np.sqrt((2*(distance/2))/JOINT_ACCELERATION_RAD)
                all_durations.extend([duration_half_distance, duration_half_distance])
            
            # Reaches max speed
            else: 
                duration_of_constant_speed = ((distance-ACCELERATION_DIST*2) / JOINT_SPEED_MAX_RAD)
                all_durations.extend([ACCELERATION_TIME, duration_of_constant_speed, ACCELERATION_TIME])

        # Calculate combined time
        all_durations.append(GRAP_TIME)
        combined_time_estimation_task = np.sum(all_durations)
        return combined_time_estimation_task, all_durations


    def __get_duration_between_positions (self, joint_positions: np.ndarray) -> np.ndarray:
        """
        Input is joins_positions of NxM, where N are the number of positions and M are DOF
        Outputs the combined duration between each joint position and the individual durations
        """
        # Get the distances of the leading axis
        distances_of_leading_axis = self.__get_distances_of_leading_axis(joint_positions)
        
        # Get the durations of movements
        durations_of_leading_axis, all_durations = self.__get_durations_of_leading_axis(distances_of_leading_axis) 
        
        # Sum the durations
        combined_duration = np.sum(durations_of_leading_axis)

        return combined_duration, all_durations

    def __get_home_ik_sol(self):
        HOME_X = TASK_CONFIG.HOME_POSITION[TASK_CONFIG.x]
        HOME_Y = TASK_CONFIG.HOME_POSITION[TASK_CONFIG.y]
        home_sol = self.robot_model.compute_joint_positions_xy(HOME_X, HOME_Y)
        return home_sol

    def __compute_thresholds (self, ik_solutions: np.ndarray):
        """Computes the thresholds corresponding to block movements
        see ____compute_ik_solutions for input format
        """
        number_of_blocks, _, _ = np.shape(ik_solutions)
        
        # Initialize threshold
        thresholds = []
        all_durations = []
    
        
        # Calculate thresholds for blocks, if there is more than one block to move
        for block_number in range(number_of_blocks):
            # Get positions which matters for the timing calculations
            # That is: All subtasks jp of the first task, and the first subtask jp of the next task
            timing_essential_positions = []
            
            # First block (or last if there are only one)
            if block_number == 0:
                home_sol = self.__get_home_ik_sol()
                origin_positions = ik_solutions[block_number, :2, :]
                timing_essential_positions = np.vstack((home_sol, origin_positions))
            
            # Middle block or last block
            else:
                joint_positions_current_block = ik_solutions[block_number-1, :, :]
                joint_positions_next_block = ik_solutions[block_number, :, :]
                before_grip_joint_position = joint_positions_current_block[-2, :] # The gripper goes up before it moves on
                first_joint_positions_of_next_block = joint_positions_next_block[:2, :] # Origin before grip and grip pos
                timing_essential_positions = np.vstack((joint_positions_current_block, before_grip_joint_position, first_joint_positions_of_next_block))
            
            # Get durations in between positions
            # combined_duration: total time between timing_essential_positions
            # durations: durations between all moves
            combined_duration, durations = self.__get_duration_between_positions(timing_essential_positions)
            thresholds.append(combined_duration)
            all_durations.extend(durations)
            
            
        return thresholds, all_durations
    
    def validate_task(self):
        """Validate the task using the task validator"""
        valid, self.task_config = self.task_validator.validate_task(self.task_config)
        if valid:
            msg = f"{MSG_TYPES.TASK_VALIDATED} {self.task_config}"

            # If task is valid, then calculate IK and thresholds
            # 1) Calculate IK
            sol = self.__compute_ik_solutions(self.task_config)
            # 2) Calculate thresholds
            thresholds, all_durations = self.__compute_thresholds(sol)
            # 3) Update task config

        
        else:
            msg = f"{MSG_TYPES.COULD_NOT_RESOLVE} None"
        self.execute_fault_resolution(msg)
        print("DT sent new config")

    def plan_fault_resolution(self, mitigation_strategy: str) -> None:
        """Resolve the current fault"""
        # resolve the fault here based on current fault and mitigation stragety
        # if fault resolved send new data to controller
        # if fault unresovled send could not resolve fault message to controller
        # in both cases go to waiting for task to start state

        
        if self.current_fault == FAULT_TYPES.MISSING_OBJECT:
            if mitigation_strategy == TASK_CONFIG.MITIGATION_STRATEGIES.SHIFT_ORIGIN:
                # print(f"Task config before (1): \n {self.task_config}")
                # # 1) remove the blocks that have already been moved
                # for block_no in range(self.current_block):
                #     self.task_config.pop(block_no+1)
                # self.task_config[TASK_CONFIG.NO_BLOCKS] -= self.current_block
                
                print(f"Task config after (1): \n {self.task_config}")
                # 2) from the current block, change two first rows in its config to the next block
                for block_no in range(                                                     # Iterate over blocks
                    self.current_block + 1, self.task_config[TASK_CONFIG.NO_BLOCKS]-1        # ... from the next block to the last block
                ):

                    self.task_config[block_no][TASK_CONFIG.ORIGIN][TASK_CONFIG.x]   = (                      # Change the x-coordinate to the next block's x-coordinate
                        self.task_config[block_no + 1][TASK_CONFIG.ORIGIN][TASK_CONFIG.x]
                    )
                    self.task_config[block_no][TASK_CONFIG.ORIGIN][TASK_CONFIG.y]   = (                      # Change the y-coordinate to the next block's y-coordinate
                        self.task_config[block_no + 1][TASK_CONFIG.ORIGIN][TASK_CONFIG.y]
                    )

                    # Change the timing threshold to the next block's threshold
                    self.task_config[block_no][TASK_CONFIG.TIMING_THRESHOLD] = (          
                        self.task_config[block_no + 1][TASK_CONFIG.TIMING_THRESHOLD]
                    )

                # remove the last block and decrement the number of blocks
                self.task_config.pop(self.task_config[TASK_CONFIG.NO_BLOCKS] - 1)
                self.task_config[TASK_CONFIG.NO_BLOCKS] -= 1

                print(f"Task config after (2): \n {self.task_config}")

                # return fault_msg with the new task_config
                return f"{MSG_TYPES.RESOLVED} {self.task_config}"
            
            elif mitigation_strategy == TASK_CONFIG.MITIGATION_STRATEGIES.TRY_PICK_STOCK:
                # For block[j] try PICK_STOCK[i++]
                self.task_config[self.current_block+1][TASK_CONFIG.ORIGIN] = TASK_CONFIG.PICK_STOCK_COORDINATES[self.pick_stock_tried][TASK_CONFIG.ORIGIN]
                self.task_config[self.current_block+1][TASK_CONFIG.TIMING_THRESHOLD] = TASK_CONFIG.PICK_STOCK_COORDINATES[self.pick_stock_tried][TASK_CONFIG.TIMING_THRESHOLD]
                self.time_of_last_message = time.time() # Reset timer  
                self.pick_stock_tried += 1
                return f"{MSG_TYPES.RESOLVED} {self.task_config}"
                

        elif self.current_fault == FAULT_TYPES.UNKOWN_FAULT:
            pass

        

    def execute_fault_resolution(self, fault_msg) -> None:
        """Execute the fault resolution: send message to controller
        fault_msg: The fault message to send to the controller"""
        self.rmq_client_out.send_message(fault_msg, RMQ_CONFIG.DT_EXCHANGE)

    
    # TODO: add proper return type
    def analyse_data(self, data):
        """Check for faults in the data"""
        # if fault present return True, fault_type
        # else return False, FAULT_TYPES.NO_FAULT
        # Meaning of current_block: The block number currently being processed 
        # What are we timing?: The time from block k grapped until block k+1 grapped
        # TODO: Check if task is done before doing the rest! I.e. check for more blocks to move here??


        # check if object is detected
        object_detected = data["output_bit_register_66"]

        # If object_detected was the first True in a sequence of booleans, then a new object was grapped
        object_grapped = not self.last_object_detected and object_detected
        self.last_object_detected = object_detected
       
        # if object detected, reset timer
        # else check if timer has expired. If expired, return fault

        # If we grap an object, we increment the current block being processed, i.e. it is initialized from 0
        if object_grapped:
            self.current_block += 1                                 # Increment block number
            self.time_of_last_message = time.time()                 # Reset timer
            self.pick_stock_tried = 1                               # Reset pick_stock_tried
            print(f"Object grapped in block {self.current_block}")

            # If we have grapped the last object, we are done
            # go to waiting for task to start state
            if self.current_block == self.task_config[TASK_CONFIG.NO_BLOCKS]-1:
                print("Task done")
                self.state = DT_STATES.WAITING_FOR_TASK_TO_START
                print("State transition -> WAITING_FOR_TASK_TO_START")

            return False, FAULT_TYPES.NO_FAULT                      # No fault present (TODO: Not needed here?)
        
        # If we have not grapped an object, we check for timing constraints
        # it is only when the timer have expired that we report a missing object
        else:
            if (self.current_block+1 < self.task_config[TASK_CONFIG.NO_BLOCKS]) and (      # If there are more blocks to move
                time.time() - self.time_of_last_message                                   # ... time passed since last object was grapped
                > self.task_config[self.current_block+1][TASK_CONFIG.TIMING_THRESHOLD]  # ... the time has expired for next block's threshold
            ):
                print(f"Missing object {self.current_block + 1}")
                return True, FAULT_TYPES.MISSING_OBJECT                                   # ... a fault present (i.e. missing object)

        return False, FAULT_TYPES.NO_FAULT

    def check_start_bit(self, data: str) -> bool:
        """Check the digital bit of the data"""
        return data["output_bit_register_65"]

    def monitor_pt(self) -> None:
        """Monitor the PT"""
        try:
            monitor_data = self.msg_queue.get(block=False)
        except Empty:
            pass
        else:
            # check the data for faults
            # if fault, send wait to controller go to fault resolution state
            # else pass
            fault_present, fault_type = self.analyse_data(monitor_data)
            if fault_present:
                print(f"Fault present: {fault_type}")
                self.current_fault = fault_type
                self.state = DT_STATES.FAULT_RESOLUTION
                print("State transition -> FAULT_RESOLUTION")
            else:
                pass

    def shutdown(self):
        """Shutdown the digital twin"""
        self.state_machine_stop_event.set()
        self.state_machine_thread.join()
        self.rmq_client_in.stop_consuming()
        print("Digital UR shutdown")
