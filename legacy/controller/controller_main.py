from Controller import Controller

if __name__ == "__main__":
    controller = Controller()
    controller.configure_rmq_client()
    controller.robot_connect()

    while True:
        pass
