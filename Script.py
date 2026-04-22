
import time
import cv2
import numpy as np
import math
import re
import os
import csv
import matplotlib.pyplot as plt
from datetime import datetime
from OpenDJI import OpenDJI, EventListener

# כתובת IP של הרחפן
DRONE_IP = "192.168.43.51"
DEADZONE = 0.1
TARGET_LAT = 31.961145
TARGET_LON = 34.797227
STOP_DISTANCE_THRESHOLD = 0.00002

# תבנית לניתוח נתוני מיקום
NUM_REG = r'[-+]?\d+\.?\d*'
LOCATION_PATTERN = re.compile(r'{"latitude":(' + NUM_REG + r'),"longitude":(' + NUM_REG + r'),"altitude":(' + NUM_REG + r')}')

# משתנים גלובליים
current_data = {"altitude": 0.0, "latitude": 0.0, "longitude": 0.0, "yaw": 0.0}
filtered_altitude = 0.0
start_time = 0

# רשימות לאיסוף נתונים בזמן אמת
times, altitudes, distances, yaws = [], [], [], []

# יצירת תיקייה לשמירת תוצאות
results_dir = "flight_results"
if not os.path.exists(results_dir):
    os.makedirs(results_dir)

# בקר PID
class PIDController:
    def __init__(self, Kp, Ki, Kd, target):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.target = target
        self.integral = 0
        self.previous_error = 0
        self.previous_time = time.time()

    def update(self, current_value):
        current_time = time.time()
        dt = current_time - self.previous_time
        if dt <= 0:
            return 0
        error = self.target - current_value
        if abs(error) < DEADZONE:
            return 0
        self.integral += error * dt
        self.integral = max(min(self.integral, 1.0), -1.0)
        derivative = (error - self.previous_error) / dt if dt > 0 else 0
        output = (self.Kp * error) + (self.Ki * self.integral) + (self.Kd * derivative)
        self.previous_error = error
        self.previous_time = current_time
        return max(min(output, 0.1), -0.1)

# מאזינים לנתונים עם איסוף נתונים
class LocationListener(EventListener):
    def onValue(self, value):
        global current_data, filtered_altitude, start_time
        if value == "null":
            print("⚠️ נתוני מיקום חסרים")
            return
        try:
            match = LOCATION_PATTERN.fullmatch(value)
            if match:
                current_data["latitude"] = float(match.group(1))
                current_data["longitude"] = float(match.group(2))
                altitude = float(match.group(3))
                filtered_altitude = 0.98 * filtered_altitude + 0.02 * altitude
                current_data["altitude"] = filtered_altitude
                if start_time > 0:
                    times.append(time.time() - start_time)
                    altitudes.append(current_data["altitude"])
                    distance = math.sqrt((TARGET_LAT - current_data["latitude"])**2 + 
                                         (TARGET_LON - current_data["longitude"])**2) * 100000
                    distances.append(distance)
        except ValueError:
            print(f"⚠️ שגיאה בניתוח מיקום: {value}")

class YawListener(EventListener):
    def onValue(self, value):
        global current_data, start_time
        if value == "null":
            print("⚠️ נתוני Yaw חסרים")
            return
        try:
            current_data["yaw"] = float(value)
            if start_time > 0:
                yaws.append(current_data["yaw"])
        except ValueError:
            print(f"⚠️ שגיאה בניתוח Yaw: {value}")

# חישוב זווית ליעד
def calculate_bearing(lat1, lon1, lat2, lon2):
    lat1, lon1, lat2, lon2 = map(math.radians, [lat1, lon1, lat2, lon2])
    dlon = lon2 - lon1
    x = math.sin(dlon) * math.cos(lat2)
    y = math.cos(lat1) * math.sin(lat2) - (math.sin(lat1) * math.cos(lat2) * math.cos(dlon))
    bearing = math.degrees(math.atan2(x, y))
    return (bearing + 360) % 360

# התחברות והמראה
def connect_and_takeoff():
    global start_time
    try:
        drone = OpenDJI(DRONE_IP)
        drone.__enter__()
        print("🔗 מחובר לרחפן בהצלחה")
        time.sleep(1)

        print("🚀 מנסה להמריא...")
        result = drone.takeoff(True)
        print(f"🛫 תוצאה: {result}")
        if "success" not in result.lower():
            print("❌ ההמראה נכשלה!")
            drone.__exit__(None, None, None)
            return None
        
        time.sleep(5)
        drone.listen(OpenDJI.MODULE_FLIGHTCONTROLLER, "AircraftLocation3D", LocationListener())
        drone.listen(OpenDJI.MODULE_FLIGHTCONTROLLER, "CompassHeading", YawListener())
        time.sleep(1)
        
        drone.enableControl(True)
        print("🎮 שליטה מלאה הופעלה לאחר התייצבות הרחפן")
        start_time = time.time()
        return drone
    except Exception as e:
        print(f"❌ שגיאה בחיבור או המראה: {e}")
        return None

# שליטה בגובה עם PID וטיפול בשגיאות חיבור
def altitude_control_with_pid(drone, target_altitude):
    pid = PIDController(Kp=0.2, Ki=0.02, Kd=1.0, target=target_altitude)
    stabilized = False
    local_start_time = time.time()
    retries = 0
    max_retries = 3
    while not stabilized:
        if time.time() - local_start_time > 30:
            print("❌ לא הצליח להתייצב בגובה היעד תוך זמן סביר!")
            return False
        print(f"🛰 גובה נוכחי: {current_data['altitude']:.2f} מטרים")
        du = pid.update(current_data["altitude"])
        try:
            drone.move(0.0, du, 0.0, 0.0)
            retries = 0
        except ConnectionAbortedError as e:
            print(f"⚠️ שגיאת חיבור: {e}")
            retries += 1
            if retries >= max_retries:
                print("❌ יותר מדי ניסיונות כושלים, הסקריפט נעצר.")
                return False
            print(f"מנסה שוב ({retries}/{max_retries})...")
            time.sleep(1)
            continue
        time.sleep(0.03)
        if abs(current_data["altitude"] - target_altitude) < 0.05:
            stabilized = True
    print(f"✅ התייצב בגובה {target_altitude} מטר")
    for _ in range(20):
        du = pid.update(current_data["altitude"])
        try:
            drone.move(0.0, du, 0.0, 0.0)
        except ConnectionAbortedError as e:
            print(f"⚠️ שגיאת חיבור במהלך יציבות: {e}")
            return False
        time.sleep(0.03)
    return True

# סיבוב לכיוון היעד
def rotate_to_target(drone, target_yaw):
    print(f"🔄 מסובב לכיוון {target_yaw:.2f} מעלות")
    local_start_time = time.time()
    retries = 0
    max_retries = 3
    while True:
        if time.time() - local_start_time > 30:
            print("❌ לא הצליח לסובב לכיוון היעד תוך זמן סביר!")
            return False
        current_yaw = current_data["yaw"]
        error = (target_yaw - current_yaw + 180) % 360 - 180
        if abs(error) < 5:
            break
        rcw = 0.1 if error > 0 else -0.1
        try:
            drone.move(rcw, 0.0, 0.0, 0.0)
            retries = 0
        except ConnectionAbortedError as e:
            print(f"⚠️ שגיאת חיבור: {e}")
            retries += 1
            if retries >= max_retries:
                print("❌ יותר מדי ניסיונות כושלים, הסקריפט נעצר.")
                return False
            print(f"מנסה שוב ({retries}/{max_retries})...")
            time.sleep(1)
            continue
        time.sleep(0.03)
    try:
        drone.move(0.0, 0.0, 0.0, 0.0)
    except ConnectionAbortedError as e:
        print(f"⚠️ שגיאת חיבור בעת עצירה: {e}")
        return False
    print("✅ פונה לכיוון היעד")
    return True

# תנועה ליעד
def move_to_target(drone, target_lat, target_lon):
    pid = PIDController(Kp=0.2, Ki=0.02, Kd=1.0, target=2.0)
    print("➡️ נע לעבר היעד")
    local_start_time = time.time()
    retries = 0
    max_retries = 3
    while True:
        if time.time() - local_start_time > 60:
            print("❌ לא הצליח להגיע ליעד תוך זמן סביר!")
            return False
        distance = math.sqrt((target_lat - current_data["latitude"])**2 + 
                             (target_lon - current_data["longitude"])**2)
        print(f"מרחק ליעד: {distance*100000:.2f} מטר, גובה נוכחי: {current_data['altitude']:.2f} מטר")
        if distance < STOP_DISTANCE_THRESHOLD:
            break
        bearing = calculate_bearing(current_data["latitude"], current_data["longitude"], 
                                    target_lat, target_lon)
        yaw_error = (bearing - current_data["yaw"] + 180) % 360 - 180
        rcw = 0.05 if yaw_error > 0 else -0.05 if yaw_error < 0 else 0.0
        speed = min(0.1, max(0.02, distance * 100000 * 0.01))
        du = pid.update(current_data["altitude"])
        try:
            drone.move(rcw, du, 0.0, speed)
            retries = 0
        except ConnectionAbortedError as e:
            print(f"⚠️ שגיאת חיבור: {e}")
            retries += 1
            if retries >= max_retries:
                print("❌ יותר מדי ניסיונות כושלים, הסקריפט נעצר.")
                return False
            print(f"מנסה שוב ({retries}/{max_retries})...")
            time.sleep(1)
            continue
        time.sleep(0.03)
    try:
        drone.move(0.0, 0.0, 0.0, 0.0)
    except ConnectionAbortedError as e:
        print(f"⚠️ שגיאת חיבור בעת עצירה: {e}")
        return False
    print(f"✅ עצר במרחק {distance*100000:.2f} מטר מהיעד")
    return True

# שמירת נתונים, גרפים וסטטיסטיקות עם תיקונים
def save_flight_data():
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    filename_base = f"{results_dir}/flight_{timestamp}"

    # שמירת נתונים גולמיים ל-CSV עם קידוד utf-8-sig
    data_file = f"{filename_base}_data.csv"
    with open(data_file, 'w', newline='', encoding='utf-8-sig') as f:
        writer = csv.writer(f)
        writer.writerow(["זמן (שניות)", "גובה (מטר)", "מרחק ליעד (מטר)", "זווית (מעלות)"])
        min_length = min(len(times), len(altitudes), len(distances), len(yaws))
        for t, alt, dist, yaw in zip(times[:min_length], altitudes[:min_length], distances[:min_length], yaws[:min_length]):
            writer.writerow([t, alt, dist, yaw])
    print(f"נתונים גולמיים נשמרו ב: {data_file}")

    # בדיקת אורכים והתראה אם יש אי התאמה
    lengths = [len(times), len(altitudes), len(distances), len(yaws)]
    if len(set(lengths)) > 1:
        print(f"⚠️ אזהרה: אורכי מערכי הנתונים שונים: {lengths}")

    # יצירת ושמירת גרפים
    plt.rcParams['font.family'] = 'DejaVu Sans'
    plt.figure(figsize=(12, 8))
    
    min_length = min(len(times), len(altitudes))
    plt.subplot(3, 1, 1)
    plt.plot(times[:min_length], altitudes[:min_length], label="גובה")
    plt.axhline(2.0, color='r', linestyle='--', label="גובה יעד")
    plt.ylabel("גובה (מטר)")
    plt.legend()
    
    min_length = min(len(times), len(distances))
    plt.subplot(3, 1, 2)
    plt.plot(times[:min_length], distances[:min_length], label="מרחק ליעד")
    plt.axhline(STOP_DISTANCE_THRESHOLD * 100000, color='r', linestyle='--', label="סף עצירה")
    plt.ylabel("מרחק (מטר)")
    plt.legend()
    
    min_length = min(len(times), len(yaws))
    plt.subplot(3, 1, 3)
    plt.plot(times[:min_length], yaws[:min_length], label="זווית סיבוב")
    plt.ylabel("זווית (מעלות)")
    plt.xlabel("זמן (שניות)")
    plt.legend()
    
    plt.tight_layout()
    plot_file = f"{filename_base}_plots.png"
    plt.savefig(plot_file)
    plt.close()
    print(f"גרפים נשמרו ב: {plot_file}")

    # חישוב ושמירת סטטיסטיקות
    min_length = min(len(altitudes), len(distances))
    stats = {
        "זמן להגעה (שניות)": times[-1] if times else 0,
        "סטייה מרבית בגובה (מטר)": max([abs(a - 2.0) for a in altitudes[:min_length]]) if altitudes else 0,
        "מרחק ממוצע (מטר)": np.mean(distances[:min_length]) if distances else 0,
        "תאריך ושעה": timestamp
    }
    stats_file = f"{results_dir}/flight_stats.csv"
    file_exists = os.path.isfile(stats_file)
    with open(stats_file, 'a', newline='', encoding='utf-8-sig') as f:
        writer = csv.DictWriter(f, fieldnames=stats.keys())
        if not file_exists:
            writer.writeheader()
        writer.writerow(stats)
    print(f"סטטיסטיקות נשמרו ב: {stats_file}")

# פונקציה ראשית
def main():
    global start_time
    drone = connect_and_takeoff()
    if drone:
        if not altitude_control_with_pid(drone, target_altitude=2.0):
            print("❌ נכשל בהמראה, הסקריפט נעצר.")
            drone.land(True)
            time.sleep(5)
            drone.__exit__(None, None, None)
            return
        
        time.sleep(2)
        if abs(current_data["altitude"] - 2.0) > 0.1:
            print(f"⚠️ גובה לא יציב ({current_data['altitude']:.2f} מטר), ממשיך להתאם...")
            if not altitude_control_with_pid(drone, target_altitude=2.0):
                print("❌ נכשל בהתאמה, הסקריפט נעצר.")
                drone.land(True)
                time.sleep(5)
                drone.__exit__(None, None, None)
                return

        bearing = calculate_bearing(current_data["latitude"], current_data["longitude"], 
                                   TARGET_LAT, TARGET_LON)
        print(f"📐 זווית ליעד: {bearing:.2f} מעלות")

        if not rotate_to_target(drone, bearing):
            print("❌ נכשל בסיבוב, הסקריפט נעצר.")
            drone.land(True)
            time.sleep(5)
            drone.__exit__(None, None, None)
            return

        if not move_to_target(drone, TARGET_LAT, TARGET_LON):
            print("❌ נכשל בתנועה ליעד, הסקריפט נעצר.")
            drone.land(True)
            time.sleep(5)
            drone.__exit__(None, None, None)
            return

        print("⬇️ יורד לגובה 1 מטר")
        if not altitude_control_with_pid(drone, target_altitude=1.0):
            print("❌ נכשל בירידה לגובה 1 מטר, הסקריפט נעצר.")
            drone.land(True)
            time.sleep(5)
            drone.__exit__(None, None, None)
            return

        print("⏸ עצירה בגובה 1 מטר לפני נחיתה")
        time.sleep(5)

        print("🛬 נוחת...")
        try:
            drone.land(True)
        except ConnectionAbortedError as e:
            print(f"⚠️ שגיאת חיבור בעת נחיתה: {e}")
        time.sleep(5)
        drone.unlisten(OpenDJI.MODULE_FLIGHTCONTROLLER, "AircraftLocation3D")
        drone.unlisten(OpenDJI.MODULE_FLIGHTCONTROLLER, "CompassHeading")
        drone.__exit__(None, None, None)
        print("🔌 נותק מהרחפן")

        # שמירת הנתונים בסיום הטיסה
        save_flight_data()

if __name__ == "__main__":
    main()
