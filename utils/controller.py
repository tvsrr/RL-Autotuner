"""
the following code implements lateral and longitudinal control using PID controllers.
longitudinal control takes set_speed, ego_speed, as input and outputs throttle and brake values.
lateral control finds the closest point on the trajectory corresponding to the current ego position, computes the heading error and cross track error, and outputs the steering value.
"""
import numpy as np

class PIDController:
    def __init__(self, kp, ki, kd, dt):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.dt = dt
        self.integral = 0.0
        self.prev_error = 0.0

    def reset(self):
        self.integral = 0.0
        self.prev_error = 0.0

    def compute(self, setpoint, measurement):
        error = setpoint - measurement
        self.integral += error * self.dt
        derivative = (error - self.prev_error) / self.dt
        output = self.kp * error + self.ki * self.integral + self.kd * derivative
        self.prev_error = error
        return output
    
class LateralController:
    def __init__(self, kp_cte, ki_cte, kd_cte, kp_heading, ki_heading, kd_heading, dt, k_cte=1.0, k_heading=1.0):
        self.pid_cte = PIDController(kp_cte, ki_cte, kd_cte, dt)
        self.pid_heading = PIDController(kp_heading, ki_heading, kd_heading, dt)
        self.k_cte = k_cte
        self.k_heading = k_heading

    def reset(self):
        self.pid_cte.reset()
        self.pid_heading.reset()

    def compute_steering(self, ego_pos, trajectory):
        """
        ego_pos: tuple of (x, y, yaw)
        trajectory: np.array of shape (N, 5) with columns [time, x, y, yaw, speed]
        """
        ego_x, ego_y, ego_yaw = ego_pos
        traj_x = trajectory[:, 1]
        traj_y = trajectory[:, 2]
        traj_yaw = trajectory[:, 3]

        # Find the closest point on the trajectory
        dists = np.sqrt((traj_x - ego_x)**2 + (traj_y - ego_y)**2)
        closest_idx = np.argmin(dists)

        # Compute heading error
        heading_error = traj_yaw[closest_idx] - ego_yaw
        heading_error = (heading_error + np.pi) % (2 * np.pi) - np.pi  # Normalize to [-pi, pi]

        # Compute cross track error and sign
        map_dx = traj_x[closest_idx] - ego_x
        map_dy = traj_y[closest_idx] - ego_y
        map_yaw = traj_yaw[closest_idx]
        perp_vec = np.array([-np.sin(map_yaw), np.cos(map_yaw)])
        ego_vec = np.array([map_dx, map_dy])
        cross_track_error = np.dot(ego_vec, perp_vec)

        # Separate PID controllers for CTE and heading error
        steering_cte = self.pid_cte.compute(0.0, cross_track_error)
        steering_heading = self.pid_heading.compute(0.0, heading_error)
        
        # Combine steering outputs
        steering = self.k_cte * steering_cte + self.k_heading * steering_heading
        steering = np.clip(steering, -1, 1)

        return steering, cross_track_error, heading_error

class LongitudinalController:
    def __init__(self, kp, ki, kd, dt):
        self.pid = PIDController(kp, ki, kd, dt)

    def reset(self):
        self.pid.reset()

    def compute_control(self, set_speed, ego_speed):
        """
        set_speed: desired speed
        ego_speed: current speed
        """
        speed_error = set_speed - ego_speed
        control = self.pid.compute(set_speed, ego_speed)
        throttle = np.clip(control, 0.0, 1.0)
        brake = np.clip(-control, 0.0, 1.0)
        return throttle, brake, speed_error
