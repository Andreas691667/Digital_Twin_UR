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
    
    a_help = cli_arguments.APPROACH_HELP
    
    key_help = cli_arguments.FILE_NAME_KEY_HELP

    parser = argparse.ArgumentParser(description='Digital UR', formatter_class=RawTextHelpFormatter)
    parser.add_argument("-ms", type=str, required=True, help=ms_help)
    parser.add_argument("-a", type=int, required=True, help=a_help)
    parser.add_argument("-key", type=str, required=False, help=key_help, default="")

    args = parser.parse_args()

    digital_ur = DigitalUR(args.ms, args.a, args.key)

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