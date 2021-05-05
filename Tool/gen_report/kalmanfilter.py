#!/usr/bin/env python3

import cv2
import numpy as np
import matplotlib.pyplot as plt
import random as ram
import pandas as pd

# df = pd.read_csv("./velocity.txt",sep= " ")
df = pd.read_csv("./Aruco2NEU.txt",sep= " ")
arr_Measurement = np.zeros((2,565))

###---Kalman filter--###
# Error estimate (t-1) :inital
vEest = 1.5
#estimate begnining (t-1) :inital
arr_Measurement[1][0] = 5
# Kalman again
vKgain = 0
# Error measurement contains : inital
vEmeasurement = 2
def update_Error_Estimate(vKg, oldEest):
	newEest = 0
	newEest = (1-vKg)*oldEest
	return newEest

def update_Kalman_Again(oldEest):
	newKg = 0
	newKg = oldEest/(oldEest+vEmeasurement)
	return newKg

def new_estimate(oldEST, KG, valueMea):
	newEST=0
	newEST=oldEST + (KG*(valueMea - oldEST))
	return newEST


for i in range(560):
	# arr_Measurement[0][i+1] = ram.randint(10,50)
	arr_Measurement[0][i+1] = df['x'][i+140]

for i in range(560):
	vKgain = update_Kalman_Again(vEest)
	arr_Measurement[1][i+1] = new_estimate(arr_Measurement[1][i], vKgain, arr_Measurement[0][i+1])
	vEest = update_Error_Estimate(vKgain, vEest)



colors = ('red', 'blue')
lab = ('measurement', 'estimate')
for i, color in enumerate(colors):
    plt.plot(arr_Measurement[i], color=color, label=lab[i])
# print(arr[0,img[8,2,2]])
# plt.imshow(img)
plt.xlabel('time')
plt.ylabel('value')
plt.legend()
plt.title('Kalman Filter')
plt.grid(color = 'green', linestyle = '--', linewidth = 0.5)
plt.show()

