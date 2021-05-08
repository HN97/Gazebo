#! /usr/bin/python3
import pandas as pd
import matplotlib.pyplot as plt
import numpy as np

df = pd.read_csv("./velocity.txt",sep= " ")
# df = pd.read_csv("./Aruco2NEU.txt",sep= " ")
df.info()

fig, (axX, axY, axZ) = plt.subplots(3, sharex=True)
axX.set(ylabel='meter (m)')
axX.set(xlabel='time')
axX.set(title='Position X')
axY.set(ylabel='meter (m)')
axY.set(xlabel='time')
axY.set(title='Position Y')
axZ.set(ylabel='meter (m)')
axZ.set(xlabel='time')
axZ.set(title='Height')
axX.plot(df['x'][:])
axY.plot(df['y'][:])
axZ.plot(df['z'][:])
axX.grid(color = 'green', linestyle = '--', linewidth = 0.5)
axY.grid(color = 'green', linestyle = '--', linewidth = 0.5)
axZ.grid(color = 'green', linestyle = '--', linewidth = 0.5)
figManager = plt.get_current_fig_manager()
figManager.resize(*figManager.window.maxsize())
plt.show()


# End of file