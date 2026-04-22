import os
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt

# Create directory to save results
save_dir = "flight_results"
os.makedirs(save_dir, exist_ok=True)

# Parameters
initial_distance = 23  # meters
initial_yaw = -55.0  # starting yaw angle
target_yaw = -135.0  # target yaw angle
cruise_altitude = 2.0  # meters
final_altitude = 1.0  # meters

drone_mass = 0.249  # kg (DJI Mini 3 Pro)
payload_mass = 0.01  # kg (10g payload)
total_mass = drone_mass + payload_mass
g = 9.81  # m/s² gravity
thrust_correction = total_mass / drone_mass  # factor to adjust thrust

drag_coefficient = 1.2
air_density = 1.225  # kg/m³
frontal_area = (9 * 3) / 10000  # converting cm² to m²

# Simulation time
sim_duration = 120  # seconds
num_points = 500

time_intervals = np.linspace(0, sim_duration, num=num_points)

takeoff_phase = int(num_points * 0.2)
yaw_correction_phase = int(num_points * 0.2)
flight_phase = int(num_points * 0.5)
landing_phase = num_points - (takeoff_phase + yaw_correction_phase + flight_phase)

# Adjusted Altitude Control with Increased Thrust
altitudes = np.linspace(0, cruise_altitude * thrust_correction, num=takeoff_phase)
altitudes = np.append(altitudes, np.full(yaw_correction_phase, cruise_altitude * thrust_correction))

yaw_values = np.full(num_points, initial_yaw)
yaw_error_values = np.full(num_points, (target_yaw - initial_yaw + 180) % 360 - 180)
current_yaw = initial_yaw

for i in range(takeoff_phase, takeoff_phase + yaw_correction_phase):
    yaw_error = (target_yaw - current_yaw + 180) % 360 - 180
    yaw_adjustment = np.sign(yaw_error) * min(abs(yaw_error) * 0.1, 2.5)  # reduced due to added inertia
    current_yaw += yaw_adjustment + np.random.normal(0, 0.1)
    current_yaw = (current_yaw + 180) % 360 - 180

    yaw_values[i] = current_yaw
    yaw_error_values[i] = yaw_error

altitudes = np.append(altitudes, np.full(flight_phase, cruise_altitude * thrust_correction))
yaw_values[takeoff_phase + yaw_correction_phase:] = target_yaw
yaw_error_values[takeoff_phase + yaw_correction_phase:] = 0

# Dynamic Speed Adjustment Considering Drag
base_speeds = np.linspace(initial_distance, 0, num=flight_phase)
drag_reduction = np.exp(-drag_coefficient * air_density * frontal_area * base_speeds)
distances = np.full(takeoff_phase + yaw_correction_phase, initial_distance)
distances = np.append(distances, base_speeds * drag_reduction)
distances = np.append(distances, np.full(landing_phase, 0))

altitudes = np.append(altitudes, np.linspace(cruise_altitude * thrust_correction, final_altitude, num=landing_phase))

sim_data_final = pd.DataFrame({
    "Time (s)": time_intervals,
    "Altitude (m)": altitudes,
    "Distance to Target (m)": distances,
    "Yaw (degrees)": yaw_values,
    "Yaw Error (degrees)": yaw_error_values
})

final_csv_path = os.path.join(save_dir, "simulated_flight_data_payload.csv")
sim_data_final.to_csv(final_csv_path, index=False)

plt.figure(figsize=(12, 8))

plt.subplot(4, 1, 1)
plt.plot(time_intervals, altitudes, label="Altitude")
plt.axhline(cruise_altitude, color='g', linestyle='--', label="Cruise Altitude (2m)")
plt.axhline(final_altitude, color='r', linestyle='--', label="Landing Altitude (1m)")
plt.ylabel("Altitude (m)")
plt.legend()

plt.subplot(4, 1, 2)
plt.plot(time_intervals, distances, label="Distance to Target")
plt.axhline(0, color='r', linestyle='--', label="Target Reached")
plt.ylabel("Distance (m)")
plt.legend()

plt.subplot(4, 1, 3)
plt.plot(time_intervals, yaw_values, label="Yaw Angle")
plt.axhline(target_yaw, color='r', linestyle='--', label="Target Yaw (-135°)")
plt.ylabel("Yaw (degrees)")
plt.legend()

plt.subplot(4, 1, 4)
plt.plot(time_intervals, yaw_error_values, label="Yaw Error")
plt.axhline(0, color='r', linestyle='--', label="Zero Error")
plt.ylabel("Yaw Error (degrees)")
plt.xlabel("Time (s)")
plt.legend()

plt.tight_layout()

final_plot_path = os.path.join(save_dir, "simulated_flight_plot_payload.png")
plt.savefig(final_plot_path)
plt.close()

print(f"✅ Updated simulation with payload effects saved in: {save_dir}")
