#https://pysource.com/2021/10/29/kalman-filter-predict-the-trajectory-of-an-object/
import cv2
import numpy as np
import copy

def setup():
    kf = cv2.KalmanFilter(4, 2)
    kf.measurementMatrix = np.array([[1, 0, 0, 0], [0, 1, 0, 0]], np.float32)
    kf.transitionMatrix = np.array([[1, 0, 1, 0], [0, 1, 0, 1], [0, 0, 1, 0], [0, 0, 0, 1]], np.float32)
    return kf

def predict(kf, coordX, coordY):
    ''' This function estimates the position of the object'''
    measured = np.array([[np.float32(coordX)], [np.float32(coordY)]])
    kf.correct(measured)
    predicted = kf.predict()
    x, y = int(predicted[0]), int(predicted[1])
    return x, y



# Kalman Filter

# The input is a 3D array with the dimensions (N of balls, N of time frames, 2 for x and y coordinates)
input = np.array([[[50,50],[40,40],[30,30],[20,20],[10,10],[0,0]],[[1,1],[2,2],[3,3],[4,4],[5,5],[6,6]]])
output = np.zeros((len(input),2))


# goes through each ball and predicts the next position for the amount of time frames
for i in range(len(input)):
    kf=setup()
    for j in range(len(input[i])):
        predicted = predict(kf,input[i][j][0],input[i][j][1])
        #print(predicted)
    output[i] = predicted


print("Predicted: ", output)