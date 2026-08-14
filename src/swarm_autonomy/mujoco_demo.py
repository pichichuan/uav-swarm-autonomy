from __future__ import annotations

import math
from dataclasses import dataclass

import mujoco
import numpy as np


@dataclass
class MujocoMissionResult:
    success: bool
    collisions: int
    dynamic_interventions: int
    elapsed_s: float


def build_xml() -> str:
    drones = "".join(
        f'''<body name="{name}" pos="0 {y} {z}"><freejoint name="{name}_free"/>
          <geom name="{name}_hull" type="sphere" size=".18" rgba="0 0 0 0" mass=".55"/>
          <geom name="{name}_crazyflie_visual" type="mesh" mesh="crazyflie_2_1" pos="0 0 -.045" rgba="{color}" contype="0" conaffinity="0"/></body>'''
        for name, y, z, color in (("uav_1", -1.1, 1.35, ".08 .75 .68 1"), ("uav_2", 1.1, 1.70, ".72 .46 .96 1"))
    )
    return f'''<mujoco model="multi_uav_autonomy"><compiler meshdir="third_party/crazyflie-simulation/meshes/stl_files"/><option timestep=".01" gravity="0 0 -9.81" integrator="implicitfast" solver="Newton"/>
      <asset><mesh name="crazyflie_2_1" file="cf2_assembly.stl" scale="4 4 4"/></asset>
      <visual><global offwidth="900" offheight="600"/><headlight ambient=".35 .35 .35" diffuse=".65 .65 .65"/></visual>
      <worldbody><light pos="0 -2 8" dir="0 0 -1" directional="true"/><geom name="ground" type="plane" size="12 12 .1" rgba=".11 .17 .23 1"/>
      {drones}
      <geom name="central_building" type="cylinder" pos="3.2 0 1.35" size=".62 1.35" rgba=".20 .28 .34 1"/>
      <geom name="central_tower_band_1" type="cylinder" pos="3.2 0 .65" size=".68 .07" rgba=".98 .68 .08 1" contype="0" conaffinity="0"/>
      <geom name="central_tower_band_2" type="cylinder" pos="3.2 0 1.65" size=".68 .07" rgba=".98 .68 .08 1" contype="0" conaffinity="0"/>
      <geom name="central_tower_beacon" type="sphere" pos="3.2 0 2.78" size=".12" rgba=".20 .95 .72 1" contype="0" conaffinity="0"/>
      <geom name="north_building" type="cylinder" pos="5.45 2.35 1.05" size=".38 1.05" rgba=".28 .36 .45 1"/>
      <geom name="north_tower_ring" type="cylinder" pos="5.45 2.35 1.45" size=".43 .06" rgba=".32 .82 .98 1" contype="0" conaffinity="0"/>
      <geom name="north_tower_beacon" type="sphere" pos="5.45 2.35 2.2" size=".10" rgba=".20 .95 .72 1" contype="0" conaffinity="0"/>
      <body name="dynamic_person" mocap="true" pos="4.35 -2.35 0"><geom name="dynamic_collision" type="cylinder" size=".25 .85" pos="0 0 .85" rgba="0 0 0 0"/>
      <geom name="person_body" type="capsule" fromto="0 0 .2 0 0 1.45" size=".16" rgba=".12 .42 .98 1" contype="0" conaffinity="0"/>
      <geom name="person_head" type="sphere" pos="0 0 1.72" size=".14" rgba=".97 .72 .52 1" contype="0" conaffinity="0"/></body>
      <geom name="goal_1" type="cylinder" pos="6.4 -.7 .01" size=".28 .01" rgba=".2 .95 .5 1" contype="0"/>
      <geom name="goal_2" type="cylinder" pos="6.4 .7 .01" size=".28 .01" rgba=".2 .95 .5 1" contype="0"/>
      </worldbody></mujoco>'''


def _paths() -> dict[str, list[np.ndarray]]:
    return {"uav_1": [np.array(point) for point in ((0, -1.1, 1.35), (2.1, -1.7, 1.35), (4.5, -1.7, 1.35), (6.4, -.7, 1.35))],
            "uav_2": [np.array(point) for point in ((0, 1.1, 1.7), (2.1, 1.7, 1.7), (4.5, 1.7, 1.7), (6.4, .7, 1.7))]}


def run_mujoco_mission(duration_s: float = 18.0, capture=None) -> MujocoMissionResult:
    model = mujoco.MjModel.from_xml_string(build_xml())
    data = mujoco.MjData(model)
    paths, indices = _paths(), {"uav_1": 1, "uav_2": 1}
    body_ids = {name: model.body(name).id for name in paths}
    dof = {name: int(model.jnt_dofadr[model.joint(f"{name}_free").id]) for name in paths}
    mass = {name: float(model.body_mass[body_ids[name]]) for name in paths}
    mocap_id = int(model.body("dynamic_person").mocapid[0])
    interventions = collisions = 0
    while data.time < duration_s:
        phase = (data.time * .8) % 8.0
        person_y = -2.35 + (phase if phase <= 4.0 else 8.0 - phase)
        person_velocity = .8 if phase <= 4.0 else -.8
        data.mocap_pos[mocap_id] = (4.35, person_y, 0)
        positions = {name: data.body(name).xpos.copy() for name in paths}
        velocities = {name: data.qvel[dof[name]:dof[name] + 3].copy() for name in paths}
        data.xfrc_applied[:] = 0.0
        commands = {}
        for name in paths:
            target = paths[name][indices[name]]
            offset = target - positions[name]
            if np.linalg.norm(offset) < .24 and indices[name] < len(paths[name]) - 1:
                indices[name] += 1
                target = paths[name][indices[name]]
                offset = target - positions[name]
            desired = 1.25 * offset / max(np.linalg.norm(offset), .001)
            predicted_person = np.array((4.35, person_y + 1.2 * person_velocity, positions[name][2]))
            separation = positions[name] - predicted_person
            distance = np.linalg.norm(separation[:2])
            active = distance < 1.45
            if active:
                desired += 1.6 * separation / max(distance, .1)
                interventions += 1
            for peer, peer_position in positions.items():
                if peer != name:
                    delta = positions[name] - peer_position
                    peer_distance = np.linalg.norm(delta)
                    if peer_distance < 1.15:
                        desired += 1.4 * delta / max(peer_distance, .1)
            speed = np.linalg.norm(desired)
            if speed > 1.45:
                desired *= 1.45 / speed
            commands[name] = desired
        for name in paths:
            # Translational PD controller.  This is the only actuation path:
            # MuJoCo integrates the free rigid bodies and resolves contacts.
            position, velocity = positions[name], velocities[name]
            horizontal = 3.2 * (commands[name][:2] - velocity[:2])
            horizontal = np.clip(horizontal, -5.5, 5.5)
            vertical = 8.0 * (paths[name][indices[name]][2] - position[2]) - 4.5 * velocity[2]
            force = mass[name] * np.array((horizontal[0], horizontal[1], 9.81 + vertical))
            data.xfrc_applied[body_ids[name], :3] = force
        if capture is not None:
            capture(model, data, active)
        mujoco.mj_step(model, data)
        # Contacts are evaluated by MuJoCo, rather than inferred from planned poses.
        for contact_index in range(data.ncon):
            contact = data.contact[contact_index]
            first = model.geom(int(contact.geom1)).name or ""
            second = model.geom(int(contact.geom2)).name or ""
            if (first.startswith("dynamic_") or second.startswith("dynamic_")) and (first.startswith("uav_") or second.startswith("uav_")):
                collisions += 1
    final_positions = {name: data.body(name).xpos.copy() for name in paths}
    success = all(np.linalg.norm(final_positions[name] - paths[name][-1]) < .48 for name in paths) and collisions == 0
    return MujocoMissionResult(success, collisions, interventions, round(float(data.time), 2))
