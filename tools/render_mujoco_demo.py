from __future__ import annotations

import sys
from pathlib import Path

import mujoco
from PIL import Image, ImageDraw

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))
from swarm_autonomy.mujoco_demo import run_mujoco_mission

frames = []
camera = mujoco.MjvCamera()
camera.type = mujoco.mjtCamera.mjCAMERA_FREE
camera.lookat[:] = (3.3, 0.0, .7)
camera.distance, camera.azimuth, camera.elevation = 9.0, 90, -48
renderer = None
count = 0
def capture(model, data, active):
    global renderer, count
    count += 1
    if count % 12: return
    if renderer is None: renderer = mujoco.Renderer(model, height=600, width=900)
    renderer.update_scene(data, camera=camera)
    image = Image.fromarray(renderer.render())
    draw = ImageDraw.Draw(image)
    draw.text((20, 18), "MuJoCo multi-UAV mapping / perception / avoidance | " + ("PEER ALTITUDE DECONFLICTION" if active else "COORDINATED FLIGHT"), fill="white")
    frames.append(image)
result = run_mujoco_mission(capture=capture)
if renderer: renderer.close()
output = ROOT / "assets" / "mujoco_swarm_avoidance.gif"
output.parent.mkdir(exist_ok=True)
frames[0].save(output, save_all=True, append_images=frames[1:], duration=80, loop=0, optimize=True)
print(result)
print(output)
