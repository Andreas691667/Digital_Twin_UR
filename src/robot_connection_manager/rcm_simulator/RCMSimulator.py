import sys
sys.path.append('../..')
sys.path.append("../../urinterface/src")
from rmq.RMQClient import Client
from config.rmq_config import RMQ_CONFIG
from config.msg_config import MSG_TYPES_MONITOR_TO_DT
import pandas as pd
from time import sleep
from robot_connection_manager.RobotConnectionManager import RobotConnectionManager
from robot_connection_manager.Controller.Controller import Controller
from queue import Queue
from threading import Event

class RCMSimulator(Controller):
    """Class to publish data to RabbitMQ server. 
     It will read data from a simulation file and publish it to the RabbitMQ server. 
     It will publish data to the specified topic"""
    
    def __init__(self, sim_file, task_config, frequency=20):
        self.simulation_file = f"simulation_data/{sim_file}"
        self.rmq_client_in = Client(host=RMQ_CONFIG.RMQ_SERVER_IP)
        self.rmq_client_out_monitor = Client(host=RMQ_CONFIG.RMQ_SERVER_IP)
        self.rmq_client_out_monitor.configure_outgoing_channel(RMQ_CONFIG.MONITOR_EXCHANGE, RMQ_CONFIG.FANOUT)
        self.rmq_client_out_controller = Client(host=RMQ_CONFIG.RMQ_SERVER_IP)
        self.shutdown_event = Event()
        self.configure_rmq_clients()
        self.controller_queue = Queue()
        self.task_config = None
        self.load_task(f"../../config/tasks/{task_config}.yaml")
        self.publish_frequency = frequency # Hz
        self.__read_data()
        self.publish_task_to_DT()
        
    def __read_data(self):
        """Read data from the simulation file."""
        self.df = pd.read_csv(self.simulation_file, delimiter=' ')
        self.len_df = len(self.df)

    def start_publishing(self):
        """Start publishing data to the RabbitMQ server."""
        self.__publish_data()

    def __publish_data(self):
        """Publish data to the RabbitMQ server."""
        while True:
            for index, row in self.df.iterrows():
                data = row.to_list()
                self.rmq_client_out_monitor.send_message(f'{MSG_TYPES_MONITOR_TO_DT.MONITOR_DATA} {data}', "MONITOR_EXCHANGE")
                print(f"\t\rPublished message [{index}/{self.len_df-1}]", end='', flush=True)
                sleep(1/self.publish_frequency)
            break
        print("\nData publishing completed")
        self.shutdown()

    def shutdown(self):
        """Shutdown the simulator."""
        print("Shutting down")
        self.rmq_client_in.stop_consuming()
        exit(0)

# main
if __name__ == "__main__":
    simulator = RCMSimulator("sq_1_5_miss_old_robot_output.csv", "square", frequency=80)
    sleep(3)
    print("Starting data publisher")
    simulator.start_publishing()
    