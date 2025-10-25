from metadrive.envs.metadrive_env import MetaDriveEnv

env = MetaDriveEnv({"use_render": True, "map": "SSS"})

def reset_env():
    return env.reset()

def step_env(steer, throttle, final_pose_reached=False):
    # Env.step may return more than four items; unpack only first four
    result = env.step([steer, throttle])
    obs, reward, done, info = result[:4]
    if done and final_pose_reached:
        env.reset()
    return obs, reward, done, info


def get_feedback():
    ego_vehicle = env.agent
    position = ego_vehicle.position
    x,y = position[0], position[1]
    yaw = ego_vehicle.heading_theta
    speed = ego_vehicle.speed
    return {"x": x, "y": y, "yaw": yaw, "speed": speed}

def close_env():
    env.close()

def show_state():
    print(get_feedback())
