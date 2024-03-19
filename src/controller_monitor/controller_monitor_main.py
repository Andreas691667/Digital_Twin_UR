from ControllerMonitor import ControllerMonitor
import argparse
import msvcrt

if __name__ == "__main__":

    parser = argparse.ArgumentParser(description='Controller Monitor')
    parser.add_argument('-t', help='The name of the task to monitor', required=True)
    parser.add_argument('-home', type=bool, help='Go to home')
    args = parser.parse_args()
    task_name = args.t
    home = args.home

    cm = ControllerMonitor(task_name, home)

    while True:
        try:
            k = msvcrt.getwche()
            if k == "c":
                break
            k = "a"
        except KeyboardInterrupt:
            break

    cm.shutdown_event.set()
