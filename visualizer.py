from ursina import *
import pandas as pd
import random
import os
from pathlib import Path

app = Ursina()

BASE_DIR = Path(__file__).resolve().parent

# We tell Ursina: "The assets are here"
application.asset_folder = BASE_DIR 

# DATA LOADING
DATA_PATH = BASE_DIR / "data" / "flight_data.csv"
if not DATA_PATH.exists():
    print(f"ERROR: Data file not found at {DATA_PATH}")
else:
    data = pd.read_csv(str(DATA_PATH))


# DATA & STATE
countdown_start = -3.0
sim_time = countdown_start
max_flight_time = data['t'].max()
running = False

# CONFIGURATION
LAUNCH_PAD_HEIGHT = 0.01
VISUAL_SCALE = 0.1

smoke_puffs = []
trail_timer = 0

# Create world
Entity(model='plane', scale=1000, texture='grass', rotation=(0,0,0)) # Ground surface
Sky = Sky()

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
    global sim_time, running

    # space for launch
    if key == 'space':
        running = True
    
    # R for Reset
    if key == 'r':
        running = False
        sim_time = countdown_start
        rocket.position=(0,LAUNCH_PAD_HEIGHT,0)
        rocket.rotation=(0,0,90 - 89)
        ground_spill.enabled = False
        for p in smoke_puffs:
            destroy(p)
        smoke_puffs.clear()
        hud_text.text = "T+: 0 s\nAltitude: 0.0 m\nVelocity: 0.0 m/s\nG-Force: 0.0"

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
            current_data = data[data['t'] <= sim_time]
            if not current_data.empty:
                row = current_data.iloc[-1]

                # Update the position
                rocket.x =  row['pos_x'] * VISUAL_SCALE
                rocket.y = (row['alt'] * VISUAL_SCALE) + LAUNCH_PAD_HEIGHT
                rocket.z = row['pos_z'] * VISUAL_SCALE

                # Update the rockets rotation
                rocket.rotation_z = 0 #90 -row['theta']
                rocket.rotation_x = row['pitch']
                rocket.rotation_y = row['yaw']

                display_time = max(0, sim_time)

                # DYNAMIC FLAME
                if 'thrust' in row and row['thrust'] > 0:
                    flame.enabled = True
                    flicker = random.uniform(0.9, 1.1)

                    # Map thrust to visibile height (increase 0.05 to 0.1 for more drama)
                    max_plume_length = (row['thrust'] * 0.15) * flicker

                    dist_to_pad = rocket.y - LAUNCH_PAD_HEIGHT

                    flame.scale_y = max(0.1, min(max_plume_length, dist_to_pad))
                    flame.scale_x = 0.4 + (row['thrust'] * 0.02)

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
                    peak_thrust = max(data['thrust'])
                    thrust_ratio = row['thrust'] / peak_thrust

                    if thrust_ratio > 0.8:
                        flame.color = color.white
                    else:
                        flame.color = lerp(color.orange, color.yellow, thrust_ratio)            
                else: 
                    flame.enabled = False
                    ground_spill.enabled = False
                # Update HUD
                # Divide Acceleration by Gravity to G
                g_force = row['accel'] / 9.81

                if row['recov_d'] != True:
                    hud_text.text = (
                        f"T+: {display_time:.2f} s\n"
                        f"Altitude: {row['alt']:.1f} m\n"
                        f"Velocity: {row['vel_m']:.1f} m/s\n"
                        f"G-Force: {g_force:.1f} G"
                    )
                    hud_text.color = color.white
                else: 
                    hud_text.text = (
                        f"T+: {display_time:.2f} s\n"
                        f"Altitude: {row['alt']:.1f} m\n"
                        f"Velocity: {row['vel_m']:.1f} m/s\n"
                        f"G-Force: {g_force:.1f} G\n"
                        f"Parachute status: DEPLOYED"
                    )
                    hud_text.color = color.white


                trail_timer += time.dt
                if trail_timer > 0.05 and 'thrust' in row and row['thrust'] > 0:
                    puff = Entity(
                        model='sphere',
                        color=color.smoke,
                        position=rocket.position,
                        scale=0.2,
                        alpha=0.8
                    )
                    smoke_puffs.append(puff)
                    trail_timer = 0

                for p in smoke_puffs:
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