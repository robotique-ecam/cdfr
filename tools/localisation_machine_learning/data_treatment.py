import numpy as np
import math
import os
import time
import pandas as pd
from sklearn.svm import NuSVR
from sklearn.pipeline import make_pipeline
from sklearn.preprocessing import StandardScaler
import joblib

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
target = x
vlx_array = np.array([vlx_0x30, vlx_0x31, vlx_0x32, vlx_0x33, vlx_0x34, vlx_0x35]).T
clf = make_pipeline(StandardScaler(), NuSVR(C=200.0, nu=1.0))

clf.fit(vlx_array, target)

joblib.dump(clf, 'test.sav')

print(clf.score(vlx_array, target))
