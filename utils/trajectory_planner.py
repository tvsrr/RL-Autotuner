import numpy as np
from scipy.interpolate import CubicSpline

def plan_trajectory(start_pos, end_pos, num_points=50, end_time=12.0):
    """
    inputs 
    start_pos : tuple of (x,y,yaw,start_speed)
    end_pos : tuple of (x,y,yaw,end_speed)
    
    outputs
    trajectory : np.array of shape (num_points, 5) with columns [time, x, y, yaw, speed]
    
    we use a cubic spline from start position to end position and it has to be time parametrized from [0,T]
    """
    x0, y0, yaw0, v0 = start_pos
    x1, y1, yaw1, v1 = end_pos

    # Time vector
    t = np.linspace(0, end_time, num_points)

    # Create cubic splines for x and y
    cs_x = CubicSpline([0, end_time], [x0, x1], bc_type=((1, v0 * np.cos(yaw0)), (1, v1 * np.cos(yaw1))))
    cs_y = CubicSpline([0, end_time], [y0, y1], bc_type=((1, v0 * np.sin(yaw0)), (1, v1 * np.sin(yaw1))))

    # Evaluate splines at time points
    x_traj = cs_x(t)
    y_traj = cs_y(t)

    # Compute yaw and speed along the trajectory
    dx_dt = cs_x.derivative()(t)
    dy_dt = cs_y.derivative()(t)
    yaw_traj = np.arctan2(dy_dt, dx_dt)
    
    #clip yaw to [-pi, pi]
    yaw_traj = (yaw_traj + np.pi) % (2 * np.pi) - np.pi

    speed_traj = np.sqrt(dx_dt**2 + dy_dt**2)

    trajectory = np.vstack((t, x_traj, y_traj, yaw_traj, speed_traj)).T
    return trajectory

