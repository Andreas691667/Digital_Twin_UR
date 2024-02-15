import sys
import json
import threading
from queue import Queue, Empty
import time

sys.path.append("..")
from rmq.RMQClient import Client
from config.rmq_config import RMQ_CONFIG
from digitalur_states import DT_STATES
from digitalur_fault_types import FAULT_TYPES
from config.msg_config import MSG_TYPES, MSG_TOPICS


class DigitalUR:
    """Class for the digital twin of the UR3e Robot"""

    def __init__(self):
        self.state: DT_STATES = DT_STATES.INITIALIZING
        self.rmq_client_in = Client(host=RMQ_CONFIG.RMQ_SERVER_IP)
        self.rmq_client_out = Client(host=RMQ_CONFIG.RMQ_SERVER_IP)
        self.msg_queue = Queue()
        self.state_machine_stop_event = threading.Event()
        self.state_machine_thread = threading.Thread(target=self.state_machine)

        self.current_fault: FAULT_TYPES = None
        self.state_machine_thread.start()

        # parameters for detecting a fault
        self.time_of_last_message = 0
        self.time_of_last_message_threshold = 3  # seconds
        self.last_object_detected = False

    def configure_rmq_clients(self):
        """Configures rmq_client_in to receive data from monitor
        and rmq_client_out to send data to controller"""
        self.rmq_client_in.configure_incoming_channel(
            self.on_monitor_message, RMQ_CONFIG.MONITOR_EXCHANGE, RMQ_CONFIG.FANOUT
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
                self.resolve_fault()

    def resolve_fault(self) -> None:
        """Resolve the fault"""
        # resolve the fault here
        # if fault resolved send new data to controller
        # if fault unresovled send could not resolve fault message to controller
        # in both cases go to waiting for task to start state
        if self.current_fault == FAULT_TYPES.MISSING_OBJECT:
            # send message to controller to stop program
            msg = f"{MSG_TYPES.STOP_PROGRAM} None"
            self.rmq_client_out.send_message(msg, RMQ_CONFIG.DT_EXCHANGE)
        elif self.current_fault == FAULT_TYPES.UNKOWN_FAULT:
            pass

        self.state = DT_STATES.WAITING_FOR_TASK_TO_START

    # TODO: add proper return type
    def check_for_faults(self, data):
        """Check for faults in the data"""
        # check for faults here
        # if fault present return True, fault_type
        # else return False, None
        # Check if timer has expired

        # check if object is detected
        object_detected = data["output_bit_register_66"]

        # If object_detected was the first True in a sequence of booleans
        object_grapped = not self.last_object_detected and object_detected

        self.last_object_detected = object_detected
        # if object detected, reset timer
        # else check if timer has expired. If expired, return fault
        if object_grapped:
            # If an object was detected then
            self.time_of_last_message = time.time()
            return False, FAULT_TYPES.NO_FAULT
        else:
            if (
                time.time() - self.time_of_last_message
                > self.time_of_last_message_threshold
            ):
                return True, FAULT_TYPES.MISSING_OBJECT
            
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
            fault_present, fault_type = self.check_for_faults(monitor_data)
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
