from time import sleep
from ControllerMonitor import ControllerMonitor
import msvcrt

if __name__ == "__main__":

    cm = ControllerMonitor()
    cm.start_monitoring()

    sleep(0.5)

    print("Ready to load program")

    while True:
        try:
            k = msvcrt.getwche()
            if k == "c":
                break
            elif k in {"1", "2"}:
                if k == "2":
                    cm.initialize_task_registers()
                    cm.load_program("/move_registers.urp")
                    cm.play_program()
            # reset k
            k = "a"
        except KeyboardInterrupt:
            break

    cm.shutdown()
