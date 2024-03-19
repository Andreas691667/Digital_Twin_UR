from DigitalUR import DigitalUR
import cli_arguments
import msvcrt
import argparse
from argparse import RawTextHelpFormatter

if __name__ == "__main__":

    ms_help = f"""The name of the mitigation strategy to use 
     \tOptions: 
     \t- {cli_arguments.SHIFT} : {cli_arguments.SHIFT_HELP} 
     \t- {cli_arguments.STOCK} : {cli_arguments.STOCK_HELP}
     \t- {cli_arguments.NONE} : {cli_arguments.NONE_HELP}"""

    parser = argparse.ArgumentParser(description='Digital UR', formatter_class=RawTextHelpFormatter)
    parser.add_argument("-ms", type=str, required=True, help=ms_help)

    args = parser.parse_args()

    mitigation_strategy = args.ms

    digital_ur = DigitalUR(mitigation_strategy)
    digital_ur.start_consuming()

# while until keyboard interrupt
    while True:
        try:
            k = msvcrt.getwche()
            if k == "c":
                break
            k = "a"
        except KeyboardInterrupt:
            break

    digital_ur.shutdown()