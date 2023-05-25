import cv2
import numpy as np


more_steps = 3


def setup_kf(init_coord_0, init_coord_1):
    kf = cv2.KalmanFilter(4, 2)
    kf.measurementMatrix = np.array([[1, 0, 0, 0], [0, 1, 0, 0]], np.float32)
    kf.transitionMatrix = np.array([[1, 0, 1, 0], [0, 1, 0, 1], [0, 0, 1, 0], [0, 0, 0, 1]], np.float32)
    coord_x_0, coord_y_0 = init_coord_0
    coord_x_1, coord_y_1 = init_coord_1
    dx_1, dy_1 = coord_x_1 - coord_x_0, coord_y_1 - coord_y_0

    # Initial position and velocity: [x, y, dx, dy]
    initial_state = np.array([[coord_x_1], [coord_y_1], [dx_1], [dy_1]], np.float32)
    kf.statePre = initial_state
    kf.statePost = initial_state
    kf.processNoiseCov = np.array([[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]], np.float32) * 1e-2
    kf.measurementNoiseCov = np.array([[1, 0], [0, 1]], np.float32) * 1e-2
    return kf


def kalman_filter(inputs):
    output = []
    for input in inputs:
        if len(input) == 0:
            continue
        predicted_steps = []
        predicted = None
        if input.shape[0] <= 1:
            output.append(np.array(input, dtype=np.float32))
            continue
        kf = setup_kf(input[0], input[1])
        for coord in input[2:]:
            coord_x, coord_y = coord
            measured = np.array([[coord_x], [coord_y]], dtype=np.float32)
            predicted = kf.predict()
            kf.correct(measured)
        predicted = kf.predict()
        predicted_steps.append(np.array(predicted[:2]).reshape((2,)))
        for _ in range(more_steps - 1):
            predicted = kf.predict()
            predicted_steps.append(np.array(predicted[:2]).reshape((2,)))
        predicted_steps = np.array(predicted_steps, dtype=np.float32)
        output.append(predicted_steps)
    return output

