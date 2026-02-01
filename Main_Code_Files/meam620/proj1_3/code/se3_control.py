import numpy as np
from scipy.spatial.transform import Rotation

class SE3Control(object):
    def __init__(self, quad_params):
        """
        Initializes the quadrotor parameters and controller gains.

        Parameters:
            quad_params: A dictionary containing quadrotor physical parameters.
        """
        # Quadrotor physical parameters.
        self.mass            = quad_params['mass']  # kg
        self.Ixx             = quad_params['Ixx']   # kg*m^2
        self.Iyy             = quad_params['Iyy']   # kg*m^2
        self.Izz             = quad_params['Izz']   # kg*m^2
        self.arm_length      = quad_params['arm_length']  # meters
        self.rotor_speed_min = quad_params['rotor_speed_min']  # rad/s
        self.rotor_speed_max = quad_params['rotor_speed_max']  # rad/s
        self.k_thrust        = quad_params['k_thrust']  # N/(rad/s)^2
        self.k_drag          = quad_params['k_drag']    # Nm/(rad/s)^2

        # Quadrotor inertia matrix
        self.inertia = np.diag([self.Ixx, self.Iyy, self.Izz])  
        self.g = 9.81  # Gravity m/s^2

        # Control gains
        self.kp = np.array([6.5, 6.5, 8.0])  
        self.kd = np.array([5.0, 5.0, 8.0])  
        self.kr = np.array([4400.0, 4400.0, 170.0])  
        self.kw = np.array([200.0, 200.0, 50.0]) 

        #self.kr = np.array([8600.0, 8600.0, 50.0])  # Attitude gains
        #self.kw = np.array([850.0, 850.0, 50.0])  # Angular velocity gains

    def update(self, t, state, flat_output):
        """
        Computes control inputs given the current state and desired trajectory.

        Inputs:
            t: Current time in seconds.
            state: Dictionary describing the present state.
            flat_output: Dictionary describing the desired trajectory.

        Outputs:
            control_input: Dictionary describing computed control inputs.
        """
        # Extracting current state
        x, v, q, w = state['x'], state['v'], state['q'], state['w']
        x_des, x_dot_des, x_ddot_des = flat_output['x'], flat_output['x_dot'], flat_output['x_ddot']
        yaw_des, yaw_dot_des = flat_output['yaw'], flat_output['yaw_dot']

        # Position control
        pos_error = x_des - x
        vel_error = x_dot_des - v

        x_ddot_cmd = x_ddot_des + self.kd * vel_error + self.kp * pos_error

        # Desired force in world frame
        F_des = self.mass * (x_ddot_cmd + np.array([0, 0, self.g]))

        # Desired orientation
        b3_des = F_des / np.linalg.norm(F_des)
        a_psi = np.array([np.cos(yaw_des), np.sin(yaw_des), 0])
        b2_des = np.cross(b3_des, a_psi)
        b2_des /= np.linalg.norm(b2_des)
        b1_des = np.cross(b2_des, b3_des)

        R_des = np.column_stack((b1_des, b2_des, b3_des))
        R_current = Rotation.from_quat(q).as_matrix()


        # Orientation control
        R_err = R_des.T @ R_current - R_current.T @ R_des
        e_R = 0.5 * np.array([R_err[2, 1], R_err[0, 2], R_err[1, 0]])
        e_omega = w - np.zeros(3)  # Desired angular velocity is zero
        u2 = self.inertia @ (-np.diag(self.kr) @ e_R - np.diag(self.kw) @ e_omega)

        # Computing thrust command
        u1 = np.dot(R_current[:, 2], F_des)


        # Computing motor speeds
        l = self.arm_length
        k_t = self.k_thrust
        k_d = self.k_drag

        # Total thrust, Torque about x-axis, Torque about y-axis, Torque about z-axis
        M = np.array([
            [k_t, k_t, k_t, k_t],        
            [0, l * k_t, 0, -l * k_t],    
            [-l * k_t, 0, l * k_t, 0],    
            [k_d, -k_d, k_d, -k_d]       
        ])

        forces = np.array([u1, u2[0], u2[1], u2[2]])
        motor_speeds_squared = np.linalg.solve(M, forces)
        motor_speeds = np.sqrt(np.clip(motor_speeds_squared, self.rotor_speed_min**2, self.rotor_speed_max**2))

        # control inputs
        control_input = {
            'cmd_motor_speeds': motor_speeds,
            'cmd_thrust': u1,
            'cmd_moment': u2,
            'cmd_q': Rotation.from_matrix(R_des).as_quat()
        }

        return control_input
