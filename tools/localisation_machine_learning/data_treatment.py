import numpy as np
import math
import os
import time
import pandas as pd

colnames = ["x", "y", "thetha", "vlx_0x30", "vlx_0x31", "vlx_0x32",  "vlx_0x33", "vlx_0x34",  "vlx_0x35"]

data = pd.read_csv('data/sector1_orient0.csv', names=colnames)
x = data.x.tolist()
y = data.y.tolist()
theta = data.thetha.tolist()
vlx_0x30 = data.vlx_0x30.tolist()
vlx_0x31 = data.vlx_0x31.tolist()
vlx_0x32 = data.vlx_0x32.tolist()
vlx_0x33 = data.vlx_0x33.tolist()
vlx_0x34 = data.vlx_0x34.tolist()
vlx_0x35 = data.vlx_0x35.tolist()
