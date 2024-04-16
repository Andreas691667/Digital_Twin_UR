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
    
    a_help = f"""The name of the approach to use 
     \tOptions: 
     \t- {cli_arguments.APPROACH} : {cli_arguments.APPROACH_HELP}"""

    parser = argparse.ArgumentParser(description='Digital UR', formatter_class=RawTextHelpFormatter)
    parser.add_argument("-ms", type=str, required=True, help=ms_help)
    parser.add_argument("-a", type=int, required=True, help=a_help)

    args = parser.parse_args()

    mitigation_strategy = args.ms
    approach = args.a

    digital_ur = DigitalUR(mitigation_strategy, approach)
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