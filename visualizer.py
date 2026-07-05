from ursina import *
import pandas as pd
import numpy as np
import random
import os
import sys
from pathlib import Path

app = Ursina()

BASE_DIR = Path(__file__).resolve().parent

# We tell Ursina: "The assets are here"
application.asset_folder = BASE_DIR

# DATA LOADING
DATA_PATH = BASE_DIR / "data" / "flight_data.csv"
if not DATA_PATH.exists():
    # Bug 1: previously the code kept running with `data` undefined and
    # crashed at `data['t'].max()`. Fail gracefully instead.
    print(f"ERROR: Data file not found at {DATA_PATH}")
    print("Cannot start playback without flight data. Exiting.")
    sys.exit(1)

data = pd.read_csv(str(DATA_PATH))


# DATA & STATE
countdown_start = -3.0
sim_time = countdown_start
max_flight_time = data['t'].max()
running = False

# CONFIGURATION
LAUNCH_PAD_HEIGHT = 0.01
VISUAL_SCALE = 0.1
MAX_SMOKE_PUFFS = 100  # Bug 7: cap the number of live smoke entities

# Bug 4: precompute numpy arrays once so per-frame lookup is O(log n) via
# np.searchsorted instead of filtering the whole DataFrame every frame.
t_arr = data['t'].to_numpy()
alt_arr = data['alt'].to_numpy()
pos_x_arr = data['pos_x'].to_numpy()
pos_z_arr = data['pos_z'].to_numpy()
vel_m_arr = data['vel_m'].to_numpy()
accel_arr = data['accel'].to_numpy()
pitch_arr = data['pitch'].to_numpy()
yaw_arr = data['yaw'].to_numpy()
thrust_arr = data['thrust'].to_numpy()

# Bug 6: normalize recov_d to a real boolean array up front (pandas may load
# it as bool or as the strings "True"/"False").
def _to_bool(v):
    if isinstance(v, str):
        return v.strip().lower() in ('true', '1', 'yes')
    return bool(v)

recov_arr = np.array([_to_bool(v) for v in data['recov_d'].to_numpy()], dtype=bool)

# Bug 5: hoist peak thrust out of the per-frame update loop.
peak_thrust = float(thrust_arr.max()) if len(thrust_arr) else 0.0

smoke_puffs = []
trail_timer = 0

# Create world
Entity(model='plane', scale=1000, texture='grass', rotation=(0,0,0)) # Ground surface
sky = Sky()  # Bug 2: don't shadow the Sky class

# ROCKET
rocket = Entity(
    model = 'assets/models/rocket.obj',
    texture = 'assets/textures/rocket_skin.png',
    color=color.black,
    scale=1,
    origin_y=-0.4
)

# FLAME
flame = Entity(
    model='quad',
    texture='circle',
    color=color.orange,
    parent=rocket,
    scale=(0.5, 0, 0.5),
    position=(0,0,0),
    origin_y=0.5,
    billboard=True,
    add_to_scene_entities=False
    )

# Ground blast
ground_spill = Entity(
    model='circle',
    color=color.orange,
    scale=0,
    position=(0, 0.02, 0),
    rotation_x=90
)

# Launch Rail
Launch_rail = Entity(
    model = 'assets/models/Launch_Rail.obj',
    texture = 'assets/textures/metal.jpg',
    position = (0.1, 0.01, 0),
    scale = 1
)
Launch_rail.rotation_z = 90 - 89

# HUD ELEMENTS
hud_text = Text(
    text='T+: 0\nAltitude: 0m\nVelocity: 0m/s\nG-Force: 0',
    position=(-0.7, 0.45),
    scale=1.5,
    color=color.white
)

# INPUT HANDLING
def input(key):
    global sim_time, running, trail_timer

    # space for launch
    if key == 'space':
        running = True
    
    # R for Reset
    if key == 'r':
        running = False
        sim_time = countdown_start
        trail_timer = 0
        rocket.position=(0,LAUNCH_PAD_HEIGHT,0)
        rocket.rotation=(0,0,90 - 89)
        # Bug 8: also reset flame / ground_spill / hud color so a reset during
        # flight doesn't leave stale visual state around.
        flame.enabled = False
        flame.color = color.orange
        ground_spill.enabled = False
        ground_spill.scale = 0
        for p in smoke_puffs:
            destroy(p)
        smoke_puffs.clear()
        hud_text.text = "T+: 0 s\nAltitude: 0.0 m\nVelocity: 0.0 m/s\nG-Force: 0.0"
        hud_text.color = color.white

# Update loop (Like in Arduino)
def update():
    global trail_timer, sim_time, running, hud_text

    if running:
        if sim_time < max_flight_time:
            sim_time += time.dt

        if sim_time < 0:
            # Countdown phase
            hud_text.text= f"T- {abs(sim_time):.1f}s\nSTATUS: ARMED"
            hud_text.color = color.yellow
            rocket.y = LAUNCH_PAD_HEIGHT
        else:
            # Flight Phase
            # Bug 4: O(log n) lookup of the current row via searchsorted on the
            # time-sorted array, replacing the per-frame DataFrame filter.
            # side='right' - 1 gives the last index where t_arr[i] <= sim_time,
            # matching the old `data[data['t'] <= sim_time].iloc[-1]`.
            idx = int(np.searchsorted(t_arr, sim_time, side='right')) - 1
            if idx >= 0:
                # Current-row scalar values pulled from the precomputed arrays.
                cur_pos_x = pos_x_arr[idx]
                cur_alt = alt_arr[idx]
                cur_pos_z = pos_z_arr[idx]
                cur_pitch = pitch_arr[idx]
                cur_yaw = yaw_arr[idx]
                cur_thrust = thrust_arr[idx]
                cur_accel = accel_arr[idx]
                cur_vel = vel_m_arr[idx]
                cur_recov = bool(recov_arr[idx])  # Bug 6: robust boolean

                # Update the position
                rocket.x = cur_pos_x * VISUAL_SCALE
                rocket.y = (cur_alt * VISUAL_SCALE) + LAUNCH_PAD_HEIGHT
                rocket.z = cur_pos_z * VISUAL_SCALE

                # Update the rockets rotation
                rocket.rotation_z = 0 #90 -row['theta']
                rocket.rotation_x = cur_pitch
                rocket.rotation_y = cur_yaw

                display_time = max(0, sim_time)

                # DYNAMIC FLAME
                if cur_thrust > 0:
                    flame.enabled = True
                    flicker = random.uniform(0.9, 1.1)

                    # Map thrust to visibile height (increase 0.05 to 0.1 for more drama)
                    max_plume_length = (cur_thrust * 0.15) * flicker

                    dist_to_pad = rocket.y - LAUNCH_PAD_HEIGHT

                    flame.scale_y = max(0.1, min(max_plume_length, dist_to_pad))
                    flame.scale_x = 0.4 + (cur_thrust * 0.02)

                    if rocket.y < max_plume_length:
                        ground_spill.enabled = True
                        ground_spill.position = (rocket.x, LAUNCH_PAD_HEIGHT + 0.02, rocket.z)

                        excess_energy = max(0,max_plume_length - dist_to_pad)
                        ground_spill.scale = (excess_energy * 2.5) + 0.5
                        ground_spill.alpha = max(0,1 - (dist_to_pad / 1.5))
                        ground_spill.color = flame.color
                    else:
                        ground_spill.enabled = False

                    # color transitiion: Orange (low thrust) -> Yellow -> White (Peak thrust)
                    # Bug 5: peak_thrust is precomputed once, not recomputed here.
                    thrust_ratio = cur_thrust / peak_thrust if peak_thrust else 0

                    if thrust_ratio > 0.8:
                        flame.color = color.white
                    else:
                        flame.color = lerp(color.orange, color.yellow, thrust_ratio)
                else:
                    flame.enabled = False
                    ground_spill.enabled = False
                # Update HUD
                # Divide Acceleration by Gravity to G
                g_force = cur_accel / 9.81

                if not cur_recov:  # Bug 6: proper boolean check
                    hud_text.text = (
                        f"T+: {display_time:.2f} s\n"
                        f"Altitude: {cur_alt:.1f} m\n"
                        f"Velocity: {cur_vel:.1f} m/s\n"
                        f"G-Force: {g_force:.1f} G"
                    )
                    hud_text.color = color.white
                else:
                    hud_text.text = (
                        f"T+: {display_time:.2f} s\n"
                        f"Altitude: {cur_alt:.1f} m\n"
                        f"Velocity: {cur_vel:.1f} m/s\n"
                        f"G-Force: {g_force:.1f} G\n"
                        f"Parachute status: DEPLOYED"
                    )
                    hud_text.color = color.white


                trail_timer += time.dt
                if trail_timer > 0.05 and cur_thrust > 0:
                    puff = Entity(
                        model='sphere',
                        color=color.smoke,
                        position=rocket.position,
                        scale=0.2,
                        alpha=0.8
                    )
                    smoke_puffs.append(puff)
                    trail_timer = 0
                    # Bug 7: cap live smoke entities so long burns don't
                    # accumulate unbounded; destroy the oldest beyond the cap.
                    while len(smoke_puffs) > MAX_SMOKE_PUFFS:
                        old = smoke_puffs.pop(0)
                        destroy(old)

                # Bug 3: iterate over a copy and rebuild the list so removing
                # faded puffs doesn't skip elements.
                for p in list(smoke_puffs):
                    growth = time.dt * 0.5
                    p.scale += Vec3(growth, growth, growth)
                    p.alpha -= time.dt * 0.2

                    if p.alpha <= 0:
                        smoke_puffs.remove(p)
                        destroy(p)

    else:
        rocket.y = LAUNCH_PAD_HEIGHT
        rocket.rotation_z = 90 - 89
    
    # CAMERA FOLLOW
    camera.position = lerp(camera.position, rocket.position + Vec3(0, 2, -15), time.dt * 2)
    camera.look_at(rocket)

# Camera control
EditorCamera()

app.run()