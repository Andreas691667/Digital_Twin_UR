from robot_connection_manager.RobotConnectionManager import RobotConnectionManager
import argparse

if __name__ == "__main__":

    parser = argparse.ArgumentParser(description='Controller Monitor')
    parser.add_argument('-t', help='The name of the task to monitor')
    parser.add_argument('-home', type=bool, help='Go to home')
    parser.add_argument('-key', type=str, required=False, help='Optional prefix to log files', default="")
    args = parser.parse_args()

    cm = RobotConnectionManager(args.t, args.home, args.key)
