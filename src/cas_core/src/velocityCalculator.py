import numpy as np

positions = np.array([[100,100],[90,100],[80,80],[70,70],[60,60],[50,50],[40,40],[30,30],[20,20],[10,10]])
samplingfrequency = 10 #Hz
N = 10000

def calcVelocity():
    distance = positions[-1]-positions[0]
    time = len(positions)/samplingfrequency
    velocity = distance/time
    print(velocity)

def checkCollision(p0,p1,p2):
    return abs((p2[0]-p1[0])*(p1[1]-p0[1]) - (p1[0]-p0[0])*(p2[1]-p1[1])) / np.sqrt(np.square(p2[0]-p1[0]) + np.square(p2[1]-p1[1]))


min_dist = checkCollision([0,0],positions[0],positions[1])
if min_dist <0.25:
    print('Warning collision imminent')
