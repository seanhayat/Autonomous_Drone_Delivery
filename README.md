Autonomous Drone Navigation

A Python script designed to control a DJI Mini 3 Pro using the OpenDJI library. The script performs an autonomous task where the drone takes off, navigates to a predefined target location using GPS coordinates, stabilizes at a specific altitude, and then lands. Below is a detailed documentation of what the code does, broken down into sections:

Overview: 
The goal of this project is to develop an autonomous navigation system for a drone carrying a small package. The drone must take off, stabilize at a cruising altitude of 2 meters, rotate towards a predefined location, navigate to the target while correcting the angle due to disturbances such as wind, descend to 1 meter, and wait for the package to be collected. The system uses a python code, OpenDJI  Library which enables to control the drone via PC, drone’s current GPS coordinates, a PID controller to ensure stable flight at cruising altitude and a control command to correct the angle to ensure precise navigation.
<img width="18749" height="158" alt="image" src="https://github.com/user-attachments/assets/75ef01c0-9119-4b53-9548-6bffbe7241a4" />
 • Libraries Used: time, cv2 (though not used in this script), numpy, math, re, and OpenDJI (a custom library for DJI drone control). • Key Features:  PID (Proportional-Integral-Derivative) control for altitude stabilization. GPS-based navigation to a target latitude and longitude.  Yaw (compass heading) adjustment to face the target. Error handling and timeouts for safety.



Initialization: The script starts by connecting to the drone and taking off.
Altitude Stabilization: The drone rises to 2 meters and stabilizes.
Navigation: o Calculates the direction to the target. o Rotates to face it. o Moves toward the target while maintaining altitude.
Final Descent: Drops to 1 meter, pauses, and lands.
Cleanup: Disconnects from the drone.
Key Notes • Timeouts: Each major operation (stabilization, rotation, movement) has a timeout to prevent infinite loops (30–60 seconds). • Dynamic Speed: Forward speed decreases as the drone approaches the target, ensuring a smooth stop. • Safety: The script includes error checks and lands the drone if anything fails. • Units: Distance is calculated in GPS coordinate units and roughly converted to meters for display (multiplied by 100,000).

Example Output • "🔗 Connected to drone successfully" • "🚀 Attempting takeoff... 🛫 Result: success" • "🛰 Current altitude: 1.95 meters" • "✅ Stabilized at 2 meters" • "📐 Bearing to target: 45.32 degrees" • "✅ Facing target" • "➡️ Moving toward target" • "Distance to target: 5.23 meters, Current altitude: 2.01 meters" • "✅ Stopped 1.98 meters from target" • "⬇️ Descending to 1 meter" • "🛬 Landing..."

This script provides a robust framework for autonomous drone navigation, with emphasis on stability, precision, and error handling.

How to use: 
After installing the MSDK Remote application according to the guide https://github.com/Penkov-D/DJI-MSDK-to-PC Download OpenDJI via this link and save it in the same folder with this code.We Connect the cell phone to the remote control, turn on HOTSPOT on a second cell phone and connect to it. An IP address will appear in the application, which we will enter in the global variable DRONE_IP in the code. Enter the GPS coordinates (you can get them using Google Maps, for example) of the location you want to fly the drone to. Copy the code to a Python compiler, turn on the drone and run the code(F5).
