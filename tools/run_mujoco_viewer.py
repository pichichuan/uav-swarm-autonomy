"""Open the physics-driven multi-UAV MuJoCo scene in the native viewer."""
from __future__ import annotations

import sys
import time
from pathlib import Path

import mujoco
from mujoco import viewer

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))
from swarm_autonomy.mujoco_demo import run_mujoco_mission

window = None
last = 0.0
def render(model, data, active):
    global window, last
    if window is None:
        window = viewer.launch_passive(model, data)
        with window.lock():
            window.cam.lookat[:] = (3.3, 0.0, .8)
            window.cam.distance, window.cam.azimuth, window.cam.elevation = 8.8, 90, -46
    if window.is_running() and data.time - last >= 0.02:
        window.sync()
        time.sleep(.02)
        last = data.time

result = run_mujoco_mission(capture=render)
if window is not None:
    window.close()
print(result)
