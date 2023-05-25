import numpy as np
from scipy.spatial.distance import cdist


distance_threshold = 1.0  # Hyperparameter.


def organize_frame(prev_frames, kalman_outputs, curr_frames):
    if len(kalman_outputs) == 0:
        return curr_frames.reshape(1, -1, 2)
    if curr_frames.shape[0] == 0:
        return []
    kalman_outputs = np.array([i[0] for i in kalman_outputs])
    distances = cdist(curr_frames, kalman_outputs, 'euclidean')
    reverse_distances = cdist(kalman_outputs, curr_frames, 'euclidean')
    reverse_closest_distances = np.min(reverse_distances, axis=1)
    reverse_closest_indices = np.argmin(reverse_distances, axis=1)
    prev_last_frames = np.array([prev_frame[-1, :] for prev_frame in prev_frames])
    output_frames = []
    for i in range(curr_frames.shape[0]):
        frame = curr_frames[i].reshape(1, -1)
        min_distance = np.min(distances[i])
        min_distance_index = np.argmin(distances[i])
        if min_distance <= distance_threshold and reverse_closest_indices[min_distance_index] == i:
            predecessor_frame = prev_frames[min_distance_index]
            output_frames.append(np.vstack([predecessor_frame, frame]))
        else:
            prev_step_distances = cdist(frame, prev_last_frames, 'euclidean')
            min_distance = np.min(prev_step_distances, axis=1)
            min_distance_index = np.argmin(prev_step_distances, axis=1)
            if min_distance <= distance_threshold and reverse_closest_distances[min_distance_index] >= min_distance:
                print(min_distance, min_distance_index)
            else:
                output_frames.append(np.array(frame))
    return output_frames

