from digital_ur import DigitalUR

if __name__ == "__main__":
    digital_ur = DigitalUR()
    digital_ur.configure_rmq_clients()
    digital_ur.start_consuming()

    while True:
        pass
