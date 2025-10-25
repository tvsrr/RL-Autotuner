import numpy as np

# estimate the ego motion based on the start position, absolute speed, yaw angle to compute vx and vy in the global frame, to finally output ego position tuple (x,y,yaw,vx,vy)
def estimate_ego_motion(start_pos, speed, sample_time =0.1):
    """
    inputs 
    start_pos : tuple of (x,y,yaw)
    speed : float, absolute speed of the ego vehicle

    outputs
    ego_motion : tuple of (x,y,yaw,vx,vy)
    """
    x, y, yaw = start_pos
    vx = speed * np.cos(yaw)
    vy = speed * np.sin(yaw)

    x_updated = x + vx*sample_time
    y_updated = y + vy*sample_time
    yaw_updated = yaw  # assuming constant yaw for simplicity
    return (x_updated, y_updated, yaw_updated, vx, vy)
