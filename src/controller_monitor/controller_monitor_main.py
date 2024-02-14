from time import sleep
from ControllerMonitor import ControllerMonitor

if __name__ == "__main__":
    import msvcrt

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
                    cm.load_program("/move-block.urp")
                    cm.play_program()
            
            # reset k
            k = "a"
        except KeyboardInterrupt:
            break

    cm.shutdown()
