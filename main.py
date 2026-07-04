import matplotlib.pyplot as plt # Plotting Tool
import pandas as pd
import numpy as np # Math tool
import json
import sys
import os

BASE_DIR = os.path.dirname(os.path.abspath(__file__))

# Utility function to get paths easily
def get_path(subfolder, filename):
    return os.path.join(BASE_DIR, subfolder, filename)

DIRS = ["data", "motors", "assets/models", "assets/textures", "config"]
for d in DIRS:
    os.makedirs(os.path.join(BASE_DIR, d), exist_ok=True)

class RocketSim: 
    def __init__(self, config_path):
        # Load the configuation file saved by main_gui.py
        with open(config_path, 'r') as f:
            config = json.load(f)

        # Physical Constants
        self.g = 9.81
        # Sea-level reference air density (from GUI config, default standard atmosphere)
        self.rho0 = float(config.get("Air Density (kg/m³)", 1.225))
        self.rho = self.rho0

        # --- Motor Data Loading ---
        # Get the motor file path directly from the GUI input
        motor_file = config.get("Motor File", "")
        motor_path = os.path.join(BASE_DIR, "motors", motor_file)

        if not os.path.isfile(motor_path):
            raise FileNotFoundError(
                f"Motor file not found: {motor_path}. "
                f"Check the 'Motor File' entry in the config and that the file exists in the motors/ folder."
            )

        motor_data = pd.read_csv(motor_path, skiprows=4)
        self.thrust_time = motor_data["Time (s)"].values
        self.thrust_force = motor_data["Thrust (N)"].values
        self.burn_time = self.thrust_time[-1]
        self.total_impulse = np.trapezoid(self.thrust_force, self.thrust_time)

        # Rocket specs from JSON
        self.mass_dry = float(config["Dry Mass (kg)"])
        self.mass_fueled = float(config["Propellant Mass (kg)"])
        self.current_fuel_mass = self.mass_fueled
        self.diameter = float(config["Diameter (m)"])

        # Calculate Cross Sectional area automatically from input diameter
        self.area = np.pi * (self.diameter / 2)**2 
        
        # Airframe drag coefficient (optional GUI input; 0.5 is a typical slender rocket)
        self.cd = float(config.get("Drag Coefficient", 0.5))
        self.cp_dist = float(config["CP Dist (m)"])
        self.cg_dist = float(config["CG Dist (m)"])
        self.fin_area = float(config["Fin Area (m\u00b2)"])
        
        # Environment & Launch Config
        self.rail_length = float(config.get("Rail Length (m)", 1.5))
        self.wind_speed_ms = float(config.get("Wind Speed (m/s)", 0.0))
        # Rail angle: 90 deg = perfectly vertical. Deviation from vertical is the tilt.
        self.rail_angle_deg = float(config.get("Rail Angle (°)", 90.0))

        # Recovery System from JSON
        self.parachute_diameter = float(config.get("Chute Dia (m)", 0.74))
        self.parachute_area = np.pi * (self.parachute_diameter / 2)**2
        self.parachute_cd = float(config.get("Chute Cd", 1.5))
        self.parachute_delay = float(config["Chute Delay (s)"])
        self.target_impact_velocity = 5.0 

        # Calculated Constants
        if self.mass_fueled > 0:
            self.isp = self.total_impulse / (self.mass_fueled * self.g)
        else:
            self.isp = 1.0  # Guard against division by zero for a zero-propellant config

        # Simulation State Variables
        self.dt = 0.01              
        self.time = 0
        self.altitude = 0
        self.velocity = 0
        self.acceleration = 0
        self.impact_velocity = 0
        self.decent_time = 0
        self.apogee_time = 0
        self.apogee_altitude = 0.0  # Guard: assigned when apogee is detected
        self.rail_departed = False  # Latches True once the rocket has left the rail
        self.max_shock_force_n = 0
        self.shock_factor = 1.5 
        self.max_velocity = 0
        self.max_mach = 0
        self.parachute_deployed = False

        # 6-DOF Specific Fields
        self.fin_dist_from_nost = 0.9614 # Adjusted based on actual rocket length

        # Inertia Tensor [Ixx, Iyy, Izz]
        self.I_tensor = np.array([0.094, 0.0008, 0.094])

        # Initial Orientation
        self.quat = np.array([1.0, 0.0, 0.0, 0.0])
        self.ang_vel = np.array([0.0, 0.0, 0.0]) 

        # Data Logging
        self.log = {"t": [], "alt": [], "pos_x": [], "pos_z": [],
                    "vel_m": [], "accel": [], "pitch": [], 
                    "yaw": [], "thrust": [], "fin_x": [], "fin_y": [],
                    "recov_d": []}
    
    def get_thrust(self, t):
        # Numpy interpolation to get rough values for missing  time spots for the exact simulation time
        return np.interp(t, self.thrust_time, self.thrust_force, left=0, right=0)

    def calculate_recovery_drag(self, velocity, altitude):
        if self.parachute_deployed:
            rho = self.get_air_density(altitude)
            # Drag Formula: Fd = 0.5 * rho * v^2 * Cd * A
            drag_force = 0.5 * rho * (velocity**2) * self.parachute_cd * self.parachute_area
            return drag_force
        return 0

    def get_air_density (self, altitude):
        # Constant for the standard atmosphere model
        P0 = 101325.0   # Standard pressure (Pa) at sea level
        T0 = 288.15     # Standard temperature (K) at sea level
        L = 0.0065      # Lapse rate of temperature (K/m)
        R = 287.05      # The specific gas constant for dry air (J/(kg*K))

        # Calculations for the temperature at altitude
        T = T0 - L * altitude

        # Ensure it cant go below absolute zero (safety feature)
        if T <= 0: return 0

        # Calculations for the pressure at altitude
        # Formula: P = P0 * (1 - L*h / T0)^(g / (R*L))
        pressure = P0 * (1 - (L * altitude) / T0)**(self.g / (R * L))

        # Calculate the density: rho = P / (R * T)
        rho = pressure / (R * T)

        # Scale by the user-supplied sea-level density (defaults to 1.225 = no change)
        rho *= self.rho0 / 1.225
        return rho
    
    def get_speed_of_sound(self, altitude):
        # Temperature at altitude (we are reusing it from get_air-density logic)
        T0 = 288.15
        L = 0.0065
        T = T0 - L * altitude

        gamma = 1.4
        R = 287.05

        # a = sqrt(gamma * R * T)
        return np.sqrt(gamma * R * max(T, 200)) # Cap T at 200K for safety
    
    def quat_to_matrix(self, q):
        """Converts a quaternion to 3x3 rotation matrix."""
        w, x, y, z = q
        return np.array([
            [1 - 2*y**2 - 2*z**2, 2*x*y - 2*z*w, 2*x*z + 2*y*w],
            [2*x*y + 2*z*w, 1 - 2*x**2 - 2*z**2, 2*y*z - 2*x*w],
            [2*x*z - 2*y*w, 2*y*z + 2*x*w, 1 - 2*x**2 - 2*y**2]
        ])
    
    def quat_derivative(self, q, w):
        qw, qx, qy, qz = q
        return 0.5 * np.array([-qx*w[0]-qy*w[1]-qz*w[2], qw*w[0]+qy*w[2]-qz*w[1], 
                                qw*w[1]-qx*w[2]+qz*w[0], qw*w[2]+qx*w[1]-qy*w[0]])

    def quat_to_euler(self, q):
        w, x, y, z = q
        # Clamp the arcsin argument to [-1, 1] to avoid NaN from floating point drift
        sin_pitch = np.clip(2.0 * (w*y - z*x), -1.0, 1.0)
        pitch = np.arcsin(sin_pitch)
        yaw = np.arctan2(2.0 * (w*z + x*y), 1.0 - 2.0 * (y*y + z*z))
        return np.array([pitch, yaw])

    def get_derivative_6dof(self, t, state, servo_angles):
        """
        state = [px, py, pz, vx, vy, vz, qw, qx, qy, qz, wx, wy, wz]
        servo_angles = [delta1, delta2, delta3, delta4] (From the PID)
        """

        # Extract data from state vector
        pos = state[0:3]
        vel = state[3:6]
        quat = state[6:10]
        ang_vel = state[10:13]

        # Atmospheric and Motor Data
        rho = self.get_air_density(pos[1]) # Altitude is Y
        thrust_mag = self.get_thrust(t)
        v_mag = np.linalg.norm(vel)

        # Linear Forces (Gravity + Thrust + Drag)
        # Gravity always pulls down on the Y Axis
        current_total_mass = self.mass_dry + max(0, state[13])
        F_gravity = np.array([0, -current_total_mass * self.g, 0])

        # Thrust: Transform the 'Up' vector of the rocket into world coordinates
        # Using the Quaternions (The logic for the rocket to know which way it's pointing)
        body_up = np.array([0, 1, 0])
        world_orientation = self.quat_to_matrix(quat)
        thrust_dir = world_orientation @ body_up
        F_thrust = thrust_dir * thrust_mag

        # FIN FORCES (Active part)
        # They create lift based on its servo angle
        # Each fin we need to calculate: Force = 0.5 * rho * v^2 * Area * Cl_alpha * angle
        total_torque = np.array([0.0, 0.0, 0.0])

        # Example: Pitch Torque from North/South Fins
        # Distance from CG to Fin Can is the 'lever arm'
        lever_arm = self.cg_dist - self.fin_dist_from_nost

        #Simplify: Torque = Force_from_fins * lever_arm
        # This is where the pid actually affects the rocket's path
        pitch_force = 0.5 * rho * v_mag**2 * self.fin_area * 2 * np.radians(servo_angles[0])
        total_torque[0] = pitch_force * lever_arm

        # Calculate Mass Flow Rate
        if state[13] > 0:
            mdot = -thrust_mag / (self.isp * self.g)
        else:
            mdot = 0

        # Dynamic wind calculations (Altitude based)
        altitude = max(0.1, pos[1])
        # Wind is stronger the higher you go (simply physics)
        local_wind = self.wind_speed_ms * (altitude / 10.0)**0.14

        # Rail logic
        # The rocket is only "on the rail" during the initial ascent (before it has
        # travelled the rail length and before apogee). The rail_departed flag latches
        # True in the main loop so this can never re-trigger near landing when the
        # position norm shrinks back down.
        dist_traveled = np.linalg.norm(pos)
        on_rail = (dist_traveled < self.rail_length) and (not self.rail_departed) and (self.apogee_time == 0)

        if on_rail:
            v_rel = vel # Ignore wind speed while on the rail
        else:
            v_rel = vel - np.array([local_wind, 0, 0]) # include the wind in the calculations

        # Drag Logic
        v_rel_mag = np.linalg.norm(v_rel)
        v_mag_sq = np.dot(v_rel, v_rel)

        if not self.parachute_deployed:
            current_cd = self.cd
            current_area = self.area
        else:
            current_cd = self.parachute_cd
            current_area = self.parachute_area

        drag_mag = 0.5 * rho * v_mag_sq * current_cd * current_area
        # Drag opposes the relative velocity; normalize by the norm of v_rel (not vel).
        F_drag = - (v_rel / (v_rel_mag + 1e-6)) * drag_mag

        # Acceleration
        accel = (F_gravity + F_thrust + F_drag) / current_total_mass

        if on_rail:
            # Only allow movement on the Y axis as long as we are on the rail.
            # We zero the lateral acceleration components (a pure constraint expressed
            # through the returned derivative). Because the rocket starts with zero
            # lateral velocity, this keeps vx/vz at exactly zero without mutating state.
            accel[0] = 0 # No lateral X movement
            accel[2] = 0 # No Lateral Z movement

            ang_accel = np.array([0.0, 0.0, 0.0])
        else:
            # Standard 6-DOF ang_accel
            ang_accel = total_torque / self.I_tensor


        # Return the rates of change
        # Velocity, Acceleration, Quaternion Derivative, Angular Acceleration
        return np.concatenate([vel, accel, self.quat_derivative(quat, ang_vel), ang_accel, [mdot]])

    def calculate_required_parachute(self):
        # rho at sea level is fine for landing calculations
        rho = 1.225

        # Rearrenged drag equation: A = (2 * mass * g) / (rho * Cd * v^2)
        numerator = 2 * self.mass_dry * self.g
        denominator = rho * self.parachute_cd * (self.target_impact_velocity**2)

        required_area = numerator / denominator

        # Calculate Diameter (assuming a circular parachute)
        # Area = pi * (d/2)^2 => d = 2 * sqrt(Area / pi)
        required_diameter = 2 * np.sqrt(required_area / np.pi)

        return required_area, required_diameter

    def print_report(self):
        # Calculate the required field radius based on drift
        required_radius = self.decent_time * self.wind_speed_ms

        req_area, req_diam = self.calculate_required_parachute()

        # General flight statistics
        print("\n--- FLIGHT SAFETY REPORT ---")
        print(f"Apogee: {self.apogee_altitude:.2f} m")
        print(f"Max Velocity: {self.max_velocity:.2f} m/s (Mach {self.max_mach:.2f})")
        print(f"Apogee at: T+{self.apogee_time:.2f}s")
        print(f"Estimated Descent Time: {self.decent_time:.2f} s")
        print(f"Recommended Field Diameter: {required_radius * 2:.2f} meters (at {self.wind_speed_ms}m/s wind)")
        print(f"Motor Burn Time: {self.burn_time:.2f}")


        # Recovery system
        print("\n--- RECOVERY SYSTEM ANALYSIS ---")
        print(f"Target Impact Velocity: {self.target_impact_velocity} m/s")
        print(f"Required Parachute Diameter: {req_diam:.2f} meters ({req_diam*100:.1f} cm)")

        # Compare with the simulation's current parachute logic
        if self.impact_velocity > self.target_impact_velocity:
            print(f"CRITICAL: Current impact speed ({self.impact_velocity:.2f} m/s) exceeds target of {self.target_impact_velocity:.2f}!")
            print(f"ACTION: You should upsize to at least a {req_diam*100:.1f} cm parachute.")
        else:
            print("SUCCESS: Your current recovery system is sufficient.")

        # Harness report
        print("\n--- RECOVERY HARNESS ANALYSIS ---")
        print(f"Max Opening Shock Force: {self.max_shock_force_n:.2f} N")
        # Convert Newtons to Kg/F for shopping purpose (divide Newton by g) 
        force_kgf = self.max_shock_force_n / self.g
        print(f"Max Opening Shock Force: {force_kgf:.2f} kgf")
        # Standard safety factor in rocketry is 3x to 5x
        safety_margin = 3
        required_strength = force_kgf * safety_margin
        print(f"Suggested Cord Strength ({safety_margin}x Safety): {required_strength:.2f} kgf")

        # Material Advice Logic
        print("MATERIAL ADVICE:")
        if force_kgf < 5:
            print("-> 3mm Elastic/Nylon: Sufficient for light loads. Ensure it is long to reduce snap.")
        elif 5 <= force_kgf < 15:
            print("-> 150kg-test Kevlar: Recommended. Use a 'shock absorber' loop to protect the airframe.")
        else:
            print("-> 350kg-test Kevlar: High load detected! Ensure the mount point in the motor tube is reinforced.")

        if self.parachute_delay > 2.0:
            print("NOTE: Your high shock force is due to the long deployment delay. The rocket is falling fast when the chute opens!")

        # STABILITY ANALYSIS
        print("\n--- STABILITY ANALYSIS ---")
        print(f"Center of Pressure (CP): {self.cp_dist:.3f} m")
        print(f"Center of Gravity (CG): {self.cg_dist:.3f} m")
        print(f"Static Stability Margin: {self.initial_stability_calibers:.2f} calibers")

        if self.initial_stability_calibers < 1.0:
            print("WARNING: Rocket may be unstable! Move CG forward (add nose weight).")
        elif self.initial_stability_calibers <= 2.0:
            print("SUCCESS: Stability margin is optimal (1-2 calibers).")
        elif self.initial_stability_calibers <= 3.0:
            print("ACCEPTABLE: Slightly overstable (2-3 calibers). May weathercock in wind.")
        else:
            print("ADVICE: Rocket is over stable (>3 calibers). It might 'weather cock' (tilt) heavily in wind.")

    def run(self):
        print("Running 6-DOF SITL Simulation...")
        sitl = FlightComputerSITL()

        # Stability Calibers calculation
        self.initial_stability_calibers = (self.cp_dist - self.cg_dist) / self.diameter

        # Initial State Vector:
        # [px, py, pz, vx, vy, vz, qw, qx, qy, qz, wx, wy, wz, fuel_mass]
        # We start at (0,0,0) pointing straight up (qw=1) with full fuel

        # Initial orientation from the launch rail angle.
        # Rail Angle 90 deg = perfectly vertical. The deviation from vertical is the
        # tilt, applied as a rotation about the body X axis:
        #   quat = [cos(theta/2), sin(theta/2), 0, 0]
        tilt_rad = np.radians(90.0 - self.rail_angle_deg)
        half = tilt_rad / 2.0
        init_quat = np.array([np.cos(half), np.sin(half), 0.0, 0.0])
        init_quat /= np.linalg.norm(init_quat)

        state = np.array([0.0, 0.0, 0.0,  0.0, 0.0, 0.0,
                          init_quat[0], init_quat[1], init_quat[2], init_quat[3],
                          0.0, 0.0, 0.0,  self.mass_fueled])

        self.time = 0
        self.parachute_deployed = False
        self.apogee_time = 0
        self.rail_departed = False

        self.max_velocity = 0
        self.max_mach = 0

        # Reset logs for 3D data
        self.log = {"t": [], "alt": [], "pos_x": [], "pos_z": [],
                    "vel_m": [], "accel": [], "pitch": [], 
                    "yaw": [], "thrust": [], "fin_x": [], "fin_y": [],
                    "recov_d": []}

        # Hard cap on simulated time to avoid an infinite loop for degenerate configs
        # (e.g. a motor that never lifts the rocket off the pad).
        MAX_SIM_TIME = 600.0  # seconds of simulated flight
        hit_time_cap = False

        # Simulation loop
        while self.time < 0.1 or state[1] >= 0:

            if self.time > MAX_SIM_TIME:
                hit_time_cap = True
                print(f"WARNING: Simulation exceeded the {MAX_SIM_TIME:.0f}s time cap. "
                      f"The rocket may never have left the pad (check thrust vs. mass). Aborting loop.")
                break

            # Sensor read (SITL)
            # Convert current Quaternion to Eurler so the PID can understand it 
            current_euler = self.quat_to_euler(state[6:10]) # Returns [pitch, yaw]
            target_euler = np.array([0.0, 0.0]) # Stay Vertical

            # Think (PID)
            servo_angles = sitl.compute_fin_angle(state[6:10], np.array([1.0, 0.0, 0.0, 0.0]), self.dt)
            # This is the Control Comand
            # ACT (RK4 Integration)
            k1 = self.get_derivative_6dof(self.time, state, servo_angles)
            k2 = self.get_derivative_6dof(self.time + self.dt/2, state + k1*self.dt/2, servo_angles)
            k3 = self.get_derivative_6dof(self.time + self.dt/2, state + k2*self.dt/2, servo_angles)
            k4 = self.get_derivative_6dof(self.time + self.dt, state + k3*self.dt, servo_angles)

            state += (self.dt / 6) * (k1 + 2*k2 + 2*k3 + k4)

            # Math cleanup
            # Quaternions must always have a magnitude of 1
            state[6:10] /= np.linalg.norm(state[6:10])

            # Latch rail departure once the rocket has cleared the rail during ascent.
            # This prevents the rail constraint from re-engaging near landing when the
            # position norm shrinks again.
            if (not self.rail_departed) and np.linalg.norm(state[0:3]) >= self.rail_length:
                self.rail_departed = True

            # Post Step logic (Apogee & Recovery)
            v_mag = np.linalg.norm(state[3:6])
            if v_mag > self.max_velocity:
                self.max_velocity = v_mag
                self.max_mach = v_mag / self.get_speed_of_sound(state[1])

            # Apogee detection
            if state[4] < 0 and self.apogee_time == 0:
                self.apogee_time = self.time
                self.apogee_altitude = state[1]

            # Parachute Deployment
            if self.apogee_time > 0 and (self.time - self.apogee_time) >= self.parachute_delay:
                if not self.parachute_deployed:
                    self.parachute_deployed = True
                    v_deploy = np.linalg.norm(state[3:6])
                    rho = self.get_air_density(state[1])
                    current_shock = (0.5 * rho * (v_deploy**2) * self.parachute_cd * self.parachute_area) * self.shock_factor
                    self.max_shock_force_n = abs(current_shock)

            current_accel_y = k1[4]

            # Logging
            self.log["t"].append(self.time)
            self.log["alt"].append(state[1]) # Y Up
            self.log["pos_x"].append(state[0]) # X Lateral
            self.log["pos_z"].append(state[2]) # Z Lateral
            self.log["vel_m"].append(v_mag)
            self.log["accel"].append(current_accel_y)
            self.log["pitch"].append(np.degrees(current_euler[0]))
            self.log["yaw"].append(np.degrees(current_euler[1]))
            self.log["thrust"].append(self.get_thrust(self.time))
            self.log["fin_x"].append(servo_angles[0])
            self.log["fin_y"].append(servo_angles[1])
            self.log["recov_d"].append(self.parachute_deployed)

            self.time += self.dt

            # Break if we've crashed/landed
            if state[1] < 0 and self.time > 0.5:
                self.impact_velocity = v_mag
                self.decent_time = self.time - self.apogee_time
                break

        # If we bailed out via the time cap (degenerate config that never landed),
        # still populate the summary fields with sensible values.
        if hit_time_cap:
            self.impact_velocity = float(np.linalg.norm(state[3:6]))
            self.decent_time = self.time - self.apogee_time

        print(f"6-DOF Simulation complete. Impact at {self.time:.2f}s")

    def plot(self):
        fig, axs = plt.subplots(5, 1, figsize=(8, 10), sharex=True)

        # Altitude
        axs[0].plot(self.log["t"], self.log["alt"], 'g')
        axs[0].set_ylabel("Altitude (m)")
        axs[0].grid(True)

        # Velocity Magnitude (Speed)
        axs[1].plot(self.log["t"], self.log["vel_m"], 'b')
        axs[1].set_ylabel("Velocity (m/s)")
        axs[1].grid(True)

        # Acceleration
        axs[2].plot(self.log["t"], self.log["accel"], 'r')
        axs[2].set_ylabel("Accel (m/s^2)")
        axs[2].grid(True)

        # Orientation (Pitch and Yaw)
        axs[3].plot(self.log["t"], self.log["pitch"], 'purple', label='Pitch')
        axs[3].plot(self.log["t"], self.log["yaw"], 'orange', label='Yaw')
        axs[3].set_ylabel("Orientation (deg)")
        axs[3].legend()
        axs[3].grid(True)

        if hasattr(self, 'apogee_time') and self.apogee_time > 0:
            deploy_time = self.apogee_time + self.parachute_delay
            axs[0].axvline(x=deploy_time, color='r', linestyle='--', label='Chute Deploy')
            axs[0].legend()

        axs[4].plot(self.log["t"], self.log["fin_x"], 'tab:blue', label='Fin X (Pitch)')
        axs[4].plot(self.log["t"], self.log["fin_y"], 'tab:red', label='Fin Y (Yaw)')
        axs[4].set_ylabel("Fin Angle (deg)")
        axs[4].set_xlabel("Time (s)")
        axs[4].legend(loc='upper right')
        axs[4].grid(True)

        plt.tight_layout()
        plt.show()

    def save_to_csv(self, filename="flight_data.csv"):
        target_path = os.path.join(BASE_DIR, "data", filename)
        df = pd.DataFrame(self.log)
        df.to_csv(target_path, index=False)
        print(f"\nData saved to {target_path}")

    def save_report(self):
        # Convert log to DataFrame to make calculations easier
        df = pd.DataFrame(self.log)

        report_path = os.path.join(BASE_DIR, "data", "flight_report.json")
        
        # Recalculate the recovery requirements
        req_area, req_diam = self.calculate_required_parachute()
        required_radius = self.decent_time * self.wind_speed_ms
        force_kgf = self.max_shock_force_n / self.g
        
        # Generate Material Advice String
        if force_kgf < 5:
            advice = "3mm Elastic/Nylon: Sufficient for light loads."
        elif 5 <= force_kgf < 15:
            advice = "150kg-test Kevlar: Recommended. Use a shock absorber loop."
        else:
            advice = "350kg-test Kevlar: High load! Reinforce motor tube mount."

        # Generate Stability Status
        if self.initial_stability_calibers < 1.0:
            stab_status = "DANGEROUS: Rocket may be unstable! Add nose weight."
        elif self.initial_stability_calibers <= 2.0:
            stab_status = "SAFE: Stability margin is optimal (1-2 cal)."
        elif self.initial_stability_calibers <= 3.0:
            stab_status = "ACCEPTABLE: Slightly overstable (2-3 cal). May weathercock in wind."
        else:
            stab_status = "WARNING: Overstable (>3 cal). May weathercock heavily."

        report_data = {
            # Flight Stats - Explicitly cast to float to avoid numpy errors
            "Apogee": float(round(self.apogee_altitude, 2)),
            "Max Velocity": float(round(self.max_velocity, 2)),
            "Max Mach": float(round(self.max_mach, 2)),
            "Apogee Time": float(round(self.apogee_time, 2)),
            "Descent Time": float(round(self.decent_time, 2)),
            "Burn Time": float(round(self.burn_time, 2)),
            
            # Recovery Analysis
            "Field Diameter": float(round(required_radius * 2, 2)),
            "Required Chute Dia": float(round(req_diam, 2)),
            "Impact Velocity": float(round(self.impact_velocity, 2)),
            "Recovery Success": "Yes" if self.impact_velocity <= self.target_impact_velocity else "No",
            
            # Harness Analysis
            "Max Shock Force N": float(round(self.max_shock_force_n, 2)),
            "Max Shock Force Kgf": float(round(force_kgf, 2)),
            "Suggested Cord Strength": float(round(force_kgf * 3, 2)),
            "Material Advice": advice,
            
            # Stability Analysis
            "CP": float(round(self.cp_dist, 3)),
            "CG": float(round(self.cg_dist, 3)),
            "Stability Margin": float(round(self.initial_stability_calibers, 2)),
            "Stability Status": stab_status
        }

        # Save to file
        with open(report_path, "w") as f:
            json.dump(report_data, f, indent=4)
        print("Detailed report saved to flight_report.json")

class FlightComputerSITL:
    def __init__(self):
        # PID Constants (Match with real rocket)
        self.kp = 1.1
        self.ki = 0.52
        self.kd = 0.115

        self.last_error = np.array([0.0, 0.0]) # Pitch and Yaw
        self.integral = np.array([0.0, 0.0])

    def compute_fin_angle(self, current_quat, target_quat, dt):
        """
        Mimics real rocket logic:
        1. Calculate the error betwee the current orientation and the target.
        2. Run PID.
        3. Output the servo angles (in degrees).
        """
        # Simplified from real flight code for Python
        error_pitch = target_quat[1] - current_quat[1]
        error_yaw = target_quat[2] - current_quat[2]
        error = np.array([error_pitch, error_yaw])

        self.integral += error * dt
        derivative = (error - self.last_error) / dt

        output = (self.kp * error) + (self.ki * self.integral) + (self.kd * derivative)
        self.last_error = error

        # Hardware Constraint: Servo Limit (e.g., +/- 15 degrees)
        output = np.clip(output, -15, 15)
        return output
    
    def get_servo_outputs(self, pitch_cmd, yaw_cmd, roll_cmd=0):
        # pitch_cmd and yaw_cmd comes from the PID loop

        # Standard 4 fin mixing (plus configuration)
        delta_1 = pitch_cmd + roll_cmd # North Fin
        delta_2 = yaw_cmd + roll_cmd # East Fin
        delta_3 = -pitch_cmd + roll_cmd # South Fin
        delta_4 = -yaw_cmd + roll_cmd # West Fin

        return [delta_1, delta_2, delta_3, delta_4]

if __name__ == "__main__":

    # Check if a config file is passed via command line
    config_file = sys.argv[1] if len(sys.argv) > 1 else "rocket_config.json"

    # Resolve the config path robustly: honour it as given (absolute or relative to the
    # current working directory), but fall back to the one next to this script so the
    # simulator works regardless of the directory it is launched from.
    if not os.path.isfile(config_file):
        fallback = os.path.join(BASE_DIR, os.path.basename(config_file))
        if os.path.isfile(fallback):
            config_file = fallback

    sim = RocketSim(config_file)
    sim.run()
    sim.print_report()
    sim.save_to_csv()
    sim.save_report()
    # Plotting is disabled in the script itself.
    #sim.plot()