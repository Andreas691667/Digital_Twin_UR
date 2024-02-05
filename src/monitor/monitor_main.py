from monitor import Monitor
import time

if __name__ == "__main__":
    monitor = Monitor()

    time.sleep(10)

    monitor.stop_recording()
