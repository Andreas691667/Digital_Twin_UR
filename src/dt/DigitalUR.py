import sys
import json
import threading
from queue import Queue, Empty
import time
import ast

sys.path.append("..")
from rmq.RMQClient import Client
from config.rmq_config import RMQ_CONFIG
from digitalur_states import DT_STATES
from digitalur_fault_types import FAULT_TYPES
from config.msg_config import MSG_TYPES_CONTROLLER_TO_DT, MSG_TYPES_DT_TO_CONTROLLER
from config.task_config import TASK_CONFIG
from ur3e.ur3e import UR3e
from task_validator.TaskValidator import TaskValidator


class DigitalUR:
    """Class for the digital twin of the UR3e Robot"""

    def __init__(self):
        self.state: DT_STATES = DT_STATES.INITIALIZING
        self.rmq_client_in = Client(host=RMQ_CONFIG.RMQ_SERVER_IP)
        # self.rmq_client_in_monitor = Client(host=RMQ_CONFIG.RMQ_SERVER_IP, name="monitor")
        self.rmq_client_out = Client(host=RMQ_CONFIG.RMQ_SERVER_IP)

        self.controller_msg_queue = Queue()
        self.monitor_msg_queue = Queue()
        self.state_machine_stop_event = threading.Event()
        self.state_machine_thread = threading.Thread(target=self.state_machine)

        # model of the UR3e robot
        self.robot = UR3e()

        self.current_fault: FAULT_TYPES = None
        self.state_machine_thread.start()

        # parameters for detecting a fault
        self.time_of_last_message = 0
        self.last_object_detected = False

        self.current_block = -1  # current block number being processed
        # self.task_config = TASK_CONFIG.block_config_close_blocks.copy()
        self.pick_stock_tried = 1

        self.task_validator = TaskValidator()

    def configure_rmq_clients(self):
        """Configures rmq_client_in to receive data from monitor
        and rmq_client_out to send data to controller"""

        self.rmq_client_in.configure_incoming_channel(
            self.on_controller_message, RMQ_CONFIG.CONTROLLER_EXCHANGE, RMQ_CONFIG.FANOUT, RMQ_CONFIG.DT_QUEUE_CONTROLLER
        )
        self.rmq_client_in.configure_incoming_channel(
            self.on_monitor_message, RMQ_CONFIG.MONITOR_EXCHANGE, RMQ_CONFIG.FANOUT, RMQ_CONFIG.DT_QUEUE_MONITOR, 1
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
                "actual_q2": float(data[2]),
                "actual_q3": float(data[3]),
                "actual_q4": float(data[4]),
                "actual_q5": float(data[5]),
                "actual_q6": float(data[6]),
                "output_bit_register_65": bool(data[7]),  # for start bit
                "output_bit_register_66": bool(data[8]),  # for object detection
            }
            self.monitor_msg_queue.put((msg_type, data_dict))

        except ValueError:
            pass
            # print("Invalid message received from monitor")



    def __get_message(self, msg_queue: Queue, block: bool = False):
        """Wait for a message"""
        try:
            type, data = msg_queue.get(block=block)
        except Empty:
            return None, None
        else:
            return type, data

    def state_machine(self):
        """The state machine for the digital twin"""
        while not self.state_machine_stop_event.is_set():
            if self.state == DT_STATES.INITIALIZING:
                self.configure_rmq_clients()
                time.sleep(1)
                self.start_consuming()
                self.state = DT_STATES.WAITING_TO_RECEIVE_TASK
                print("State transition -> WAITING_TO_RECEIVE_TASK")
            
            elif self.state == DT_STATES.WAITING_TO_RECEIVE_TASK:
                msg_type, msg_data = self.__get_message(self.controller_msg_queue)
                if msg_data:
                    if msg_type == MSG_TYPES_CONTROLLER_TO_DT.NEW_TASK:
                        self.task_config = ast.literal_eval(msg_data)

                        # validate the task
                        validate_msg = self.validate_task()
                        # send validated task to controller
                        self.rmq_client_out.send_message(validate_msg, RMQ_CONFIG.DT_EXCHANGE)
                        # state transition
                        self.state = DT_STATES.WAITING_FOR_TASK_TO_START
                        print("State transition -> WAITING_FOR_TASK_TO_START")

            elif self.state == DT_STATES.WAITING_FOR_TASK_TO_START:
                msg_type, msg_data = self.__get_message(self.monitor_msg_queue)
                if msg_data:
                    # check the digital bit 0 of the data
                    # if set go to monitoring state
                    # else pass
                    if self.check_start_bit(msg_data):
                        print("State transition -> MONITORING_PT")
                        self.state = DT_STATES.MONITORING_PT
                        # Start timer
                        self.time_of_last_message = time.time()

            elif self.state == DT_STATES.MONITORING_PT:
                self.monitor_pt()
            elif self.state == DT_STATES.FAULT_RESOLUTION:
                # stop program firstly
                self.execute_fault_resolution(f"{MSG_TYPES_DT_TO_CONTROLLER.WAIT} None")
                fault_msg = self.plan_fault_resolution(TASK_CONFIG.MITIGATION_STRATEGIES.SHIFT_ORIGIN)
                self.validate_task()
                self.execute_fault_resolution(fault_msg)
                self.state = DT_STATES.WAITING_FOR_TASK_TO_START
                print("State transition -> WAITING_FOR_TASK_TO_START")

    def validate_task(self):
        """Validate the task using the task validator"""
        valid, self.task_config = self.task_validator.validate_task(self.task_config)
        if valid:
            msg = f"{MSG_TYPES_DT_TO_CONTROLLER.TASK_VALIDATED} {self.task_config}"
        
        else:
            msg = f"{MSG_TYPES_DT_TO_CONTROLLER.COULD_NOT_RESOLVE} None"
        return msg

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
                
                # print(f"Task config after (1): \n {self.task_config}")
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

                # print(f"Task config after (2): \n {self.task_config}")

                # return fault_msg with the new task_config
                return f"{MSG_TYPES_DT_TO_CONTROLLER.RESOLVED} {self.task_config}"
            
            elif mitigation_strategy == TASK_CONFIG.MITIGATION_STRATEGIES.TRY_PICK_STOCK:
                # For block[j] try PICK_STOCK[i++]
                self.task_config[self.current_block+1][TASK_CONFIG.ORIGIN] = TASK_CONFIG.PICK_STOCK_COORDINATES[self.pick_stock_tried][TASK_CONFIG.ORIGIN]
                self.task_config[self.current_block+1][TASK_CONFIG.TIMING_THRESHOLD] = TASK_CONFIG.PICK_STOCK_COORDINATES[self.pick_stock_tried][TASK_CONFIG.TIMING_THRESHOLD]
                self.time_of_last_message = time.time() # Reset timer  
                self.pick_stock_tried += 1
                return f"{MSG_TYPES_DT_TO_CONTROLLER.RESOLVED} {self.task_config}"
                

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
                self.current_block = -1
                self.state = DT_STATES.WAITING_TO_RECEIVE_TASK
                print("State transition -> WAITING_TO_RECEIVE_TASK")

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

        _, monitor_data = self.__get_message(self.monitor_msg_queue)

        if monitor_data:
            fault_present, fault_type = self.analyse_data(monitor_data)
            if fault_present:
                print(f"Fault present: {fault_type}")
                self.current_fault = fault_type
                self.state = DT_STATES.FAULT_RESOLUTION
                print("State transition -> FAULT_RESOLUTION")

    def shutdown(self):
        """Shutdown the digital twin"""
        self.state_machine_stop_event.set()
        self.state_machine_thread.join()
        self.rmq_client_in.stop_consuming()
        print("Digital UR shutdown")
