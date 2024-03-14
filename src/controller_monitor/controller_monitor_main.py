from ControllerMonitor import ControllerMonitor
import argparse

if __name__ == "__main__":

    # Instantiate ControllerMonitor
    # task_name = sys.argv[1]

    parser = argparse.ArgumentParser(description='Controller Monitor')
    parser.add_argument('-t', help='The name of the task to monitor')
    args = parser.parse_args()
    task_name = args.t

    cm = ControllerMonitor(task_name)

