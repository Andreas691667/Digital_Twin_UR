from DigitalUR import DigitalUR
import msvcrt

if __name__ == "__main__":
    digital_ur = DigitalUR()
    digital_ur.start_consuming()

# while until keyboard interrupt
    while True:
        try:
            k = msvcrt.getwche()
            if k == "c":
                break
        except KeyboardInterrupt:
            break

    digital_ur.