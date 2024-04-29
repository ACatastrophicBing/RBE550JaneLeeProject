import matplotlib.pyplot as plt
import numpy as np
import matplotlib.animation as animation
import math
import json
import os
import argparse
import pandas as pdb
from itertools import chain

parser = argparse.ArgumentParser()
parser.add_argument("--data_file")


args = parser.parse_args()

print("\n. . . Live Plotting the robot path and simulation ", args.data_file, " . . .")

if args.data_file is not None:
    print(". . . Loading Model Data")
    data_path = args.data_path
