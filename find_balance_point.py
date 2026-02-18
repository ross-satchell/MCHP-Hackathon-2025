"""
Two-Wheeled Balancing Bot - CALIBRATION MODE
Measures and displays tilt angle only. 
Motors and PID are disabled to find the physical balance point.
"""
import time
import board
import adafruit_icm20x
import math

# ============================================================================
# CONFIGURATION
# ============================================================================
USE_COMPLEMENTARY_FILTER = True
COMPLEMENTARY_ALPHA = 0.98

# ============================================================================
# IMU ANGLE CALCULATION
# ============================================================================
class AngleEstimator:
    def __init__(self, use_filter=True, alpha=0.98):
        self.use_filter = use_filter
        self.alpha = alpha
        self.angle = 0.0
        self.last_time = time.monotonic()

    def calculate_accel_angle(self, accel_x, accel_z):
        # Result in degrees
        return math.atan2(accel_z, accel_x) * 180.0 / math.pi
    
    def update(self, accel_x, accel_z, gyro_y):
        current_time = time.monotonic()
        dt = current_time - self.last_time
        if dt <= 0.0: dt = 0.001
        
        accel_angle = self.calculate_accel_angle(accel_x, accel_z)
        
        if self.use_filter:
            gyro_rate = gyro_y * 180.0 / math.pi
            self.angle = self.alpha * (self.angle + gyro_rate * dt) + (1 - self.alpha) * accel_angle
        else:
            self.angle = accel_angle
        
        self.last_time = current_time
        return self.angle

# ============================================================================
# MAIN PROGRAM (CALIBRATION ONLY)
# ============================================================================
def main():
    print("\n--- Calibration Mode: Find the Balance Point ---")
    i2c = board.I2C()
    
    try:
        icm = adafruit_icm20x.ICM20948(i2c, 0x69)
    except:
        icm = adafruit_icm20x.ICM20948(i2c, 0x68)

    angle_estimator = AngleEstimator(use_filter=USE_COMPLEMENTARY_FILTER)

    print("Instructions:")
    print("1. Balance the bot with your fingers very lightly.")
    print("2. Find the point where it feels weightless (center of gravity).")
    print("3. Note the Angle below—that is your new TARGET_ANGLE.\n")
    
    time.sleep(1.0)

    try:
        while True:
            accel_x, _, accel_z = icm.acceleration
            _, gyro_y, _ = icm.gyro
            
            current_angle = angle_estimator.update(accel_x, accel_z, gyro_y)
            
            # Print the current angle clearly
            print(f"Current Tilt: {current_angle:6.2f}°")
            
            time.sleep(0.05) # 20Hz is plenty for human reading
            
    except KeyboardInterrupt:
        print("\nCalibration finished.")

if __name__ == "__main__":
    main()