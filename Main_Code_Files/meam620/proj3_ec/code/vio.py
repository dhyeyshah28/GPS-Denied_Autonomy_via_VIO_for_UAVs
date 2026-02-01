import numpy as np
from numpy.linalg import inv
from numpy.linalg import norm
from scipy.spatial.transform import Rotation
import matplotlib.pyplot as plt

# Lists to store history for plotting
wb_history = []
g_history = []

def nominal_state_update(nominal_state, w_m, a_m, dt):
    """
    function to perform the nominal state update

    :param nominal_state: State tuple (p, v, q, a_b, w_b, g)
                    all elements are 3x1 vectors except for q which is a Rotation object
    :param w_m: 3x1 vector - measured angular velocity in radians per second
    :param a_m: 3x1 vector - measured linear acceleration in meters per second squared
    :param dt: duration of time interval since last update in seconds
    :return: new tuple containing the updated state
    """
    ####
    
    #new_p = np.zeros((3, 1))
    #new_v = np.zeros((3, 1))
    #new_q = Rotation.identity()

    # Updating position
    #  p ← p + v * dt + 0.5 * (R * (a_m - a_b) + g) * dt^2

    # Unpacking nominal_state tuple
    p, v, q, a_b, w_b, g = nominal_state

    # quaternion to rotation matrix
    R = q.as_matrix()  
    accel = R @ (a_m - a_b) + g
    n_p = p + v * dt + 0.5 * accel * dt**2

    # Updating velocity
    
    #  v ← v + (R * (a_m - a_b) + g) * dt
    n_v = v + accel * dt

    # Updating quaternion
    delta_rot = Rotation.from_rotvec((w_m - w_b).ravel() * dt)

    # Quaternion multiplication
    n_q = q * delta_rot  

    # Biases and gravity remain unchanged
    return n_p, n_v, n_q, a_b, w_b, g

def error_covariance_update(nominal_state, error_state_covariance, w_m, a_m, dt,
                            accelerometer_noise_density, gyroscope_noise_density,
                            accelerometer_random_walk, gyroscope_random_walk):
    """
    Function to update the error state covariance matrix

    :param nominal_state: State tuple (p, v, q, a_b, w_b, g)
                        all elements are 3x1 vectors except for q which is a Rotation object
    :param error_state_covariance: 18x18 initial error state covariance matrix
    :param w_m: 3x1 vector - measured angular velocity in radians per second
    :param a_m: 3x1 vector - measured linear acceleration in meters per second squared
    :param dt: duration of time interval since last update in seconds
    :param accelerometer_noise_density: standard deviation of accelerometer noise
    :param gyroscope_noise_density: standard deviation of gyro noise
    :param accelerometer_random_walk: accelerometer random walk rate
    :param gyroscope_random_walk: gyro random walk rate
    :return: 18x18 covariance matrix
    """
    # Unpack nominal_state tuple
    p, v, q, a_b, w_b, g = nominal_state

    
    
    #a_m = a_m.reshape(3,)  # Flatten to 1D array
    #a_b = a_b.reshape(3,)
    #w_m = w_m.reshape(3,)
    #w_b = w_b.reshape(3,)

    # Computing R from quaternion
    R = q.as_matrix()
    
    # Computing a_m - a_b for skew-symmetric matrix

    a_diff = (a_m - a_b).reshape((3,))
    a_skew = np.array([[0, -a_diff[2], a_diff[1]],
                       [a_diff[2], 0, -a_diff[0]],
                       [-a_diff[1], a_diff[0], 0]])
    

    # Computing w_m - w_b for rotation update
    
    #w_diff_skew = np.array([[0, -w_diff[2], w_diff[1]],
    #                        [w_diff[2], 0, -w_diff[0]],
    #                        [-w_diff[1], w_diff[0], 0]])  # 3x3 skew-symmetric matrix

    w_diff = (w_m - w_b)
    w_diff_rotvec = (w_diff * dt).reshape((1, 3))
    w_diff_rot = (((Rotation.from_rotvec(w_diff_rotvec)).as_matrix()).reshape(3,3)).T


    #F_x = np.zeros((18, 18))
    #F_x[0:3, 0:3] = np.eye(3)  # I
    #F_x[0:3, 3:6] = np.eye(3) * dt  # I * dt
    # F_x[3:6, 3:6] = np.eye(3)  # I
    # F_x[3:6, 6:9] = -R @ a_skew * dt  # -R * [a_m - a_b]_x * dt
    # F_x[3:6, 9:12] = -R * dt  # -R * dt
    # F_x[3:6, 15:18] = np.eye(3) * dt  # I * dt
    # F_x[6:9, 6:9] = w_diff_rot  # R^T{(w_m - w_b) * dt}
    # F_x[6:9, 12:15] = -np.eye(3) * dt  # -I * dt
    # F_x[9:12, 9:12] = np.eye(3)  # I
    # F_x[12:15, 12:15] = np.eye(3)  # I
    #F_x[15:18, 15:18] = np.eye(3)  # I

    # Fx Matrix
    F_x = np.identity(18)
    F_x[0:3, 3:6] = np.eye(3) * dt
    F_x[3:6, 6:9] = -R @ a_skew * dt
    F_x[3:6, 9:12] = -R * dt
    F_x[3:6, 15:18] = np.eye(3) * dt
    F_x[6:9, 6:9] = w_diff_rot
    F_x[6:9, 12:15] = -np.eye(3) * dt

    # Constructing F_i matrix
    F_i = np.zeros((18, 12))
    F_i[3:15, 0:12] = np.identity(12)

    # Constructing Q_i matrix
    Q_i = np.zeros((12, 12))
    Q_i[0:3, 0:3] = accelerometer_noise_density**2 * dt**2 * np.eye(3)
    Q_i[3:6, 3:6] = gyroscope_noise_density**2 * dt**2 * np.eye(3)
    Q_i[6:9, 6:9] = accelerometer_random_walk**2 * dt * np.eye(3)
    Q_i[9:12, 9:12] = gyroscope_random_walk**2 * dt * np.eye(3)

    # Updating covariance
    nc = F_x @ error_state_covariance @ F_x.T + F_i @ Q_i @ F_i.T

    return nc


def measurement_update_step(nominal_state, error_state_covariance, uv, Pw, error_threshold, Q):
    """
    Function to update the nominal state and the error state covariance matrix based on a single
    observed image measurement uv, which is a projection of Pw.

    :param nominal_state: State tuple (p, v, q, a_b, w_b, g)
                        all elements are 3x1 vectors except for q which is a Rotation object
    :param error_state_covariance: 18x18 initial error state covariance matrix
    :param uv: 2x1 vector of image measurements
    :param Pw: 3x1 vector world coordinate
    :param error_threshold: inlier threshold
    :param Q: 2x2 image covariance matrix
    :return: new_state_tuple, new error state covariance matrix, innovation
    """
    p, v, q, a_b, w_b, g = nominal_state

    R = q.as_matrix()
    P_c0 = R.T @ (Pw - p)  # P_c0 in current left camera frame
    predicted_uv = np.array([P_c0[0], P_c0[1]]).reshape(2,1) / P_c0[2]


    #Xc = P_c0[0]
    #Yc = P_c0[1]
    #qZc = P_c0[2]

    #print("P_c0")
    #print(P_c0)

    #if abs(Zc) < 1e-6:
    #    return (p, v, q, a_b, w_b, g), error_state_covariance, np.zeros((2, 1))
    #Xc, Yc, Zc = P_c0.flatten()


    # Computing innovation
    innovation = uv - predicted_uv
    
    # Printing innovation to check its magnitude
    #print(f"Innovation norm: {norm(innovation)}")
    
    # Storing w_b and g for plotting 
    wb_history.append(w_b.flatten().copy())
    g_history.append(g.flatten().copy())
    
    if norm(innovation) >= error_threshold:
        return nominal_state, error_state_covariance, innovation
    else:
        dzt_dPc = np.array([[1, 0, -predicted_uv[0, 0]],
                            [0, 1, -predicted_uv[1, 0]]]) / np.squeeze(P_c0)[2]
        
        P_c0_skew = np.array([[0, -P_c0[2,0], P_c0[1,0]],
                              [P_c0[2,0], 0, -P_c0[0,0]],
                              [-P_c0[1,0], P_c0[0,0], 0]])
        
        H_t = np.zeros((2, 18))
        H_t[:, 0:3] = dzt_dPc @ (-R.T)
        H_t[:, 6:9] = dzt_dPc @ P_c0_skew

        S = H_t @ error_state_covariance @ H_t.T + Q.T
        K_t = error_state_covariance @ H_t.T @ inv(S)
        delta_x = K_t @ innovation
        
        delta_p = delta_x[0:3]
        delta_v = delta_x[3:6]
        delta_theta = delta_x[6:9]
        delta_a_b = delta_x[9:12]
        delta_w_b = delta_x[12:15]
        delta_g = delta_x[15:18]

        # Updating params
        new_p = p + delta_p
        new_v = v + delta_v
        delta_q = Rotation.from_rotvec(delta_theta.reshape(3))  
        new_q = q * delta_q
        new_a_b = a_b + delta_a_b
        new_w_b = w_b + delta_w_b
        new_g = g + delta_g
        
        I = np.eye(18)
        new_covariance = (I - K_t @ H_t) @ error_state_covariance @ (I - K_t @ H_t).T + K_t @ Q @ K_t.T
        
        return (new_p, new_v, new_q, new_a_b, new_w_b, new_g), new_covariance, innovation

