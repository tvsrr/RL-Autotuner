from metadrive.envs.metadrive_env import MetaDriveEnv

env = MetaDriveEnv({"use_render": True})
env.reset()
for _ in range(200):
    env.step([0.0, 1.0])
env.close()
print("✅ Ran successfully")
