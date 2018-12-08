#=======================================================================
# Name        : FrisbeeTrajectoryVisualization.py
# Author      : Jason Elsted
# Version     : 1.0
# Description : Frisbee Trajectory Visualization
#=======================================================================

import sys
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

fin = '../simulation_data/' + sys.argv[1]
pts = pd.read_csv(fin, engine = 'python')
print(pts)

fig = plt.figure()
ax = plt.axes(projection='3d')
minVal = min(min(pts['x']), min(pts['y']), min(pts['z']))
maxVal = max(max(pts['x']), max(pts['y']), max(pts['z']))
ax.set_ylim(minVal, maxVal)
ax.set_xlim(0, maxVal)
ax.set_zlim(0, maxVal)
ax.plot(np.array(pts['x']), np.array(pts['y']), np.array(pts['z']), '-b')
fout = '../trajectories/' + fin[19:-4] + '.png'
plt.savefig(fout)
plt.show()