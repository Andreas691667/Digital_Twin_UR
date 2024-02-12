# from urinterface.robot_connection import RobotConnection
from sys import path

path.append("..")
from config.robot_config import ROBOT_CONFIG
from urinterface.robot_connection import RobotConnection
from threading import Thread, Event
from pathlib import Path
from queue import Queue, Empty
from time import sleep

from rmq.RMQClient import Client
from config.rmq_config import RMQ_CONFIG


class ControllerMonitor:
    """Class responsible for all robot interaction"""

    def __init__(self) -> None:
        self.robot_connection = RobotConnection(ROBOT_CONFIG.ROBOT_HOST)
        self.conf_file = "record_configuration.xml"
        self.log_file = "test_motion1.csv"
        self.log_file_path = Path("test_results") / Path(self.log_file)

        self.rmq_client_in = Client(host=RMQ_CONFIG.RMQ_SERVER_IP)
        self.rmq_client_out = Client(host=RMQ_CONFIG.RMQ_SERVER_IP)
        self.configure_rmq_clients()

        self.controller_thread = Thread(target=self.controller_worker)
        self.controller_thread_event = Event()
        self.controller_queue = Queue()

        self.monitor_thread = Thread(
            target=self.robot_connection.start_recording(
                config_file=self.conf_file,
                filename=self.log_file_path,
                overwrite=True,
                frequency=50,
                publish_topic=["actual_q"],
                rmq_client=self.rmq_client_out
            )
        )

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
        print(f" [x] Received from DT: {body}")
        self.controller_queue.put(body)

    def start_monitoring(self):
        """start the monitor thread"""
        self.monitor_thread.start()

    def stop_monitoring(self):
        """stop the monitor thread"""
        self.robot_connection.stop_recording()
        sleep(1)
        self.monitor_thread.join()

    def load_program(self, program_name: str) -> None:
        """Load program"""
        succ = self.robot_connection.load_program(program_name)
        print(f"Program loaded: {succ}")

    def play_program(self) -> None:
        """Start loaded program"""
        program_started = self.robot_connection.play_program()
        if program_started:
            print(f"Program started: {program_started}")
            self.controller_thread.start()
        else:
            pass

    def stop_program(self) -> None:
        """stop current exection"""
        self.robot_connection.stop_program()

    def controller_worker(self):
        """worker for the controller thread.
        Listens for new control signals from the DT"""

        while not self.controller_thread_event.is_set():
            try:
                msg = self.controller_queue.get(timeout=1)
            except Empty:
                pass
            else:
                self.stop_program()

    def shutdown(self):
        """shutdown everything: robot, rmq, threads"""
        self.stop_program()
        self.stop_monitoring()
        self.controller_thread_event.set()
        self.controller_thread.join()
        self.rmq_client_in.stop_consuming()


if __name__ == "__main__":
    import msvcrt

    cm = ControllerMonitor()
    cm.start_monitoring()

    sleep(.5)

    print("Ready to load program")

    while True:
        try:
            k = msvcrt.getwche()
            if k == "c":
                break
            elif k in {"1", "2"}:
                if k == "2":
                    cm.load_program("/program1.urp")
                    cm.play_program()
            # reset k
            k = "a"
        except KeyboardInterrupt:
            break

    cm.shutdown()
