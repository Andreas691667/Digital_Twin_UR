from ControllerMonitor import ControllerMonitor
import argparse

if __name__ == "__main__":

    parser = argparse.ArgumentParser(description='Controller Monitor')
    parser.add_argument('-t', help='The name of the task to monitor')
    parser.add_argument('-home', type=bool, help='Go to home')
    args = parser.parse_args()
    task_name = args.t
    home = args.home

    cm = ControllerMonitor(task_name, home)
