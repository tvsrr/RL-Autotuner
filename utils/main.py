# Main control loop
"""
calls the meta drive wrapper, ego motion estimation, trajectory planner, controller to run a closed loop control
"""
from math import pi
from metadrive_wrapper import reset_env, step_env, get_feedback, close_env
from ego_motion_estimation import estimate_ego_motion
from trajectory_planner import plan_trajectory
from controller import LateralController, LongitudinalController
import matplotlib.pyplot as plt
import numpy as np
import time
import pdb
import matplotlib.gridspec as gridspec


# plot the planned trajetory and tracked trajectory
def plot_driving_params(driving_params):
    throttle = driving_params["throttle"]
    brake = driving_params["brake"]
    steering = driving_params["steering"]
    heading = driving_params["heading"]
    cross_track = driving_params["cross_track"]
    planned_trajectory = driving_params["trajectory"]
    tracked_trajectory = driving_params["tracked_trajectory"]
    time_vec = np.arange(len(throttle)) * 0.1  # assuming sample time of 0.1s

    plt.figure(figsize=(14, 9))
    gs = gridspec.GridSpec(3, 3, width_ratios=[1, 1, 1.8])  # right panel bigger but not dominant

    # Left panel - control/time plots
    ax1 = plt.subplot(gs[0, 0])
    ax1.plot(time_vec, throttle, color='green')
    ax1.set_title("Throttle over Time")
    ax1.set_xlabel("Time (s)"); ax1.set_ylabel("Throttle"); ax1.grid()

    ax2 = plt.subplot(gs[0, 1])
    ax2.plot(time_vec, brake, color='red')
    ax2.set_title("Brake over Time")
    ax2.set_xlabel("Time (s)"); ax2.set_ylabel("Brake"); ax2.grid()

    ax3 = plt.subplot(gs[1, 0])
    ax3.plot(time_vec, steering, color='blue')
    ax3.set_title("Steering over Time")
    ax3.set_xlabel("Time (s)"); ax3.set_ylabel("Steering"); ax3.grid()

    ax4 = plt.subplot(gs[1, 1])
    ax4.plot(time_vec, heading, color='orange')
    ax4.set_title("Heading Error over Time")
    ax4.set_xlabel("Time (s)"); ax4.set_ylabel("Heading Error (rad)"); ax4.grid()

    ax5 = plt.subplot(gs[2, 0:2])
    ax5.plot(time_vec, cross_track, color='purple')
    ax5.set_title("Cross Track Error over Time")
    ax5.set_xlabel("Time (s)"); ax5.set_ylabel("Cross Track Error (m)"); ax5.grid()

    # Right - larger trajectory plot
    ax6 = plt.subplot(gs[:, 2])
    ax6.plot(planned_trajectory[:, 1], planned_trajectory[:, 2], color='black', label='Planned', linewidth=2)
    ax6.plot(tracked_trajectory[:, 0], tracked_trajectory[:, 1], color='cyan', label='Tracked', linewidth=2)
    ax6.set_title("Planned vs Tracked Trajectory")
    ax6.set_xlabel("X Position (m)"); ax6.set_ylabel("Y Position (m)")
    ax6.legend()
    ax6.grid(True)

    # remove forced equal aspect
    ax6.set_aspect('auto')  

    plt.tight_layout()
    plt.show()


if __name__ == "__main__":
    env_obs = reset_env()
    lateral_controller = LateralController(kp_cte=1.0, ki_cte=0.0, kd_cte=0.1,
                                           kp_heading=1.0, ki_heading=0.0, kd_heading=0.1,
                                           dt=0.1, k_cte=-1.0, k_heading=-1.0)
    sample_time = 0.1  # 100 ms
    # record actual ego positions for later plotting
    tracked_trajectory = []

    # Plan a simple straight-line goal 10m ahead at current speed

    initial_feedback = get_feedback()
    x, y, yaw, vx, vy = initial_feedback["x"], initial_feedback["y"], initial_feedback["yaw"], 0, 0
    goal_yaw = 0.0    # keep heading constant for straight line
    speed_mag = 7.0
    goal_x = 50.0
    goal_y = y+5
    goal_state = (goal_x, goal_y, goal_yaw, speed_mag)
    trajectory = plan_trajectory((x, y, yaw, speed_mag), goal_state,
                                    num_points=50, end_time=sample_time * 50)
    #pdb.set_trace()
    done = False
    final_pose_reached = False
    step = 0
    distance_to_goal = np.hypot(x - goal_x, y - goal_y)
    tracked_speed = []
    throttle_values = []
    brake_values = []
    steering_values = []
    heading_errors = []
    cross_track_errors = []

    while not final_pose_reached:
        feedback = get_feedback()
        ego_pos = (feedback["x"], feedback["y"], feedback["yaw"])
        tracked_trajectory.append([ego_pos[0], ego_pos[1]])
        speed = feedback["speed"]

        print(f"Step {step}: Ego Position: {ego_pos}, Speed: {speed:.2f} m/s, Distance to Goal: {distance_to_goal:.2f} m")

        # Estimate ego motion
        ego_motion = estimate_ego_motion(ego_pos, speed, sample_time)

        #use the longitudinal controller to maintain speed
        longitudinal_controller = LongitudinalController(kp=1.0, ki=0.0, kd=0.1, dt=sample_time)
        throttle, brake, speed_error = longitudinal_controller.compute_control(speed_mag, speed)

        # Record speed, throttle, brake for analysis
        tracked_speed.append(speed)
        throttle_values.append(throttle)
        brake_values.append(brake)

        # Compute steering using lateral controller
        steering, cte, heading_error = lateral_controller.compute_steering(ego_pos, trajectory)

        # Record lateral errors for analysis
        steering_values.append(steering)
        heading_errors.append(heading_error)
        cross_track_errors.append(cte)

        # Step the environment
        env_obs, reward, done, info = step_env(steering, throttle, final_pose_reached)
        distance_to_goal = np.hypot(ego_pos[0] - goal_x, ego_pos[1] - goal_y)
        final_pose_reached = (distance_to_goal < 0.5) or (distance_to_goal > 90.0)  # also stop if we are too far away

        if final_pose_reached:
            print("Episode finished or goal reached. Resetting environment.")
            env_obs = reset_env()
        
        time.sleep(sample_time)  # Maintain the sample time
        step += 1

    # After control loop, show planned vs. tracked paths
    tracked_array = np.array(tracked_trajectory)
    driving_params = {
            "throttle": throttle_values,
            "brake": brake_values,
            "steering": steering_values,
            "heading": heading_errors,
            "cross_track": cross_track_errors,
            "trajectory": trajectory,
            "tracked_trajectory": tracked_array
        }
    plot_driving_params(driving_params)

    close_env()
