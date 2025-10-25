# MetaDrive PID‑Control

## Current status 
PID setup with the meta drive environment is done.
Setting up RL-Auto tuner is left. 
main has to be moved to src outside

Welcome! This project shows how to hook up a simple PID-based lateral controller
to the MetaDrive simulator and drive a vehicle along a smooth cubic-spline path.
Once the simulation wraps up, you'll get a Matplotlib plot comparing the planned
spline with the actual path your vehicle took.

## 🚀 Quickstart

1. Install dependencies in one go:
   ```bash
   pip install -r requirements.txt
   ```

2. Fire up the demo:
   ```bash
   python main.py
   ```

   You’ll see the MetaDrive window pop up and drive the car for 1,000 steps.
   When it’s done, a second window will appear with:
   - **Blue curve**: the final spline planned 10 m ahead at each time step
   - **Red curve**: the actual (x, y) trajectory driven by your controller

## 🧰 Dependencies

- Python ≥ 3.7
- MetaDrive simulator
- NumPy
- SciPy
- Matplotlib

## 🗂 File Overview

| Script                       | Purpose                                                        |
|:-----------------------------|:---------------------------------------------------------------|
| **main.py**                  | Main loop: reset → estimate motion → plan spline → steer → step |
| **controller.py**            | PID controllers (lateral + optional longitudinal)              |
| **ego_motion_estimation.py** | Estimate global (x,y,yaw,vx,vy) from current pose + speed      |
| **trajectory_planner.py**    | Generate time-parameterized cubic spline between two states     |
| **metadrive_wrapper.py**     | Thin wrapper for MetaDrive’s reset, step, feedback, and close  |

## 🔎 Tips & Tweaks

- Tweak the “10 m look-ahead” in `main.py` to change how far the car previews.
- Adjust PID gains in `controller.py` if the car is too wobbly or strays off track.
- To draw the spline inside MetaDrive, sprinkle in some `env.road_network.draw_polyline(...)`
  calls before `env.step()`.

---


