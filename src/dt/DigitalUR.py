import sys
import json
import threading
from queue import Queue, Empty

sys.path.append("..")
from rmq.RMQClient import Client
from config.rmq_config import RMQ_CONFIG
from digitalur_states import DT_STATES
from digitalur_fault_types import FAULT_TYPES


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

        # self.configure_rmq_clients()

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

        # format data here
        data = json.loads(body)

        # put data in the queue
        self.msg_queue.put(data)

    def state_machine(self):
        """The state machine for the digital twin"""
        while not self.state_machine_stop_event.is_set():
            if self.state == DT_STATES.INITIALIZING:
                self.configure_rmq_clients()
                self.start_consuming()
                self.state = DT_STATES.WAITING_FOR_TASK_TO_START
                print("State: WAITING_FOR_TASK_TO_START")
            elif self.state == DT_STATES.WAITING_FOR_TASK_TO_START:
                try:
                    data = self.msg_queue.get()
                except Empty:
                    pass
                else:
                    # check the digital bit 0 of the data
                    # if set go to monitoring state
                    # else pass
                    if self.check_digital_bit(data):
                        self.state = DT_STATES.MONITORING_PT
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
            pass
        elif self.current_fault == FAULT_TYPES.UNKOWN_FAULT:
            pass



    def check_for_faults(self, data) -> tuple(bool, FAULT_TYPES):
        """Check for faults in the data"""
        # check for faults here
        # if fault present return True, fault_type
        # else return False, None
        return False, None

    def check_digital_bit(self, data) -> bool:
        """Check the digital bit of the data"""

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
                self.current_fault = fault_type
                self.state = DT_STATES.FAULT_RESOLUTION
            else:
                pass
