import matplotlib.pyplot as plt
from rich.console import Console
from rich.prompt import IntPrompt
import argparse

from utils.logger_visuals import LoggerVisuals

def parse_arguments():
    parser = argparse.ArgumentParser()
    parser.add_argument('filename', help='.pkl file to check')
    args = parser.parse_args()
    return args

def query_data(logger:LoggerVisuals):
    console = Console()
    joint_id = -1
    while joint_id < 0 or joint_id > 29:
        joint_id = IntPrompt.ask(
            'Enter joint idx that you want to see',
            console=console,
            default=0,
            show_default=False,
        )
        if joint_id < 0 or joint_id > 29:
            print('JOINT ID MUST BE FROM 0 TO 29')

    logger.plot_targets(joint_id)
    # while not plt.waitforbuttonpress(0):
        # pass
    

def main():
    args = parse_arguments()
    filename = args.filename
    logger = LoggerVisuals(
        local_load_file=filename,
        dump_on_death=False
    )
    while True:
        query_data(logger)
        
if __name__ == '__main__':
    main()