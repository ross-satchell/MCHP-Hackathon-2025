"""
Two-Wheeled Balancing Bot with PID Control
Uses ICM20948 IMU and Drok dual motor driver
"""
import time
import board
import pwmio
import digitalio
import adafruit_icm20x
import math

# ============================================================================
# CONFIGURATION
# ============================================================================

# Debug options
DEBUG = True
USE_COMPLEMENTARY_FILTER = True  # Set to False to use only accelerometer

# PID Tuning Parameters (Start conservative and tune)
KP = 40.0   # Proportional gain - how aggressively to respond to current error
KI = 0.5    # Integral gain - eliminates steady-state error
KD = 1.5    # Derivative gain - dampens oscillations

# Motor constraints
MAX_PWM = 65535      # Maximum PWM value (16-bit)
MIN_PWM = 8000       # Minimum PWM to overcome motor friction
MOTOR_DEADBAND = 3.0 # Angle deadband (degrees) where motors stay off

# Angle limits for safety
MAX_ANGLE = 45.0     # If tilt exceeds this, stop trying (fallen over)

# Filter parameters
COMPLEMENTARY_ALPHA = 0.98  # Weight for gyro vs accel (0.98 = 98% gyro, 2% accel)

# Target angle (degrees from vertical)
TARGET_ANGLE = 0.0

# ============================================================================
# MOTOR CONTROL CLASS
# ============================================================================

class DrokMotorDriver:
    """Controls two DC motors via Drok driver board"""
    
    def __init__(self, in1_pin, in2_pin, ena1_pin, in3_pin, in4_pin, ena2_pin):
        """Initialize motor driver pins"""
        # Motor 1 pins
        self.in1 = digitalio.DigitalInOut(in1_pin)
        self.in1.direction = digitalio.Direction.OUTPUT
        self.in2 = digitalio.DigitalInOut(in2_pin)
        self.in2.direction = digitalio.Direction.OUTPUT
        self.ena1 = pwmio.PWMOut(ena1_pin, frequency=20000, duty_cycle=0)
        
        # Motor 2 pins
        self.in3 = digitalio.DigitalInOut(in3_pin)
        self.in3.direction = digitalio.Direction.OUTPUT
        self.in4 = digitalio.DigitalInOut(in4_pin)
        self.in4.direction = digitalio.Direction.OUTPUT
        self.ena2 = pwmio.PWMOut(ena2_pin, frequency=20000, duty_cycle=0)
        
        self.brake()
    
    def brake(self):
        """Brake both motors"""
        self.in1.value = False
        self.in2.value = False
        self.in3.value = False
        self.in4.value = False
        self.ena1.duty_cycle = 0
        self.ena2.duty_cycle = 0
    
    def set_motor1(self, speed):
        """
        Set motor 1 speed and direction
        speed: -65535 (full reverse) to +65535 (full forward)
        """
        if speed > 0:  # Forward
            self.in1.value = True
            self.in2.value = False
            self.ena1.duty_cycle = min(int(abs(speed)), MAX_PWM)
        elif speed < 0:  # Reverse
            self.in1.value = False
            self.in2.value = True
            self.ena1.duty_cycle = min(int(abs(speed)), MAX_PWM)
        else:  # Brake
            self.in1.value = False
            self.in2.value = False
            self.ena1.duty_cycle = 0
    
    def set_motor2(self, speed):
        """
        Set motor 2 speed and direction
        speed: -65535 (full reverse) to +65535 (full forward)
        """
        if speed > 0:  # Forward
            self.in3.value = True
            self.in4.value = False
            self.ena2.duty_cycle = min(int(abs(speed)), MAX_PWM)
        elif speed < 0:  # Reverse
            self.in3.value = False
            self.in4.value = True
            self.ena2.duty_cycle = min(int(abs(speed)), MAX_PWM)
        else:  # Brake
            self.in3.value = False
            self.in4.value = False
            self.ena2.duty_cycle = 0
    
    def set_both_motors(self, speed):
        """Set both motors to same speed (for balancing)"""
        self.set_motor1(speed)
        self.set_motor2(speed)

# ============================================================================
# PID CONTROLLER CLASS
# ============================================================================

class PIDController:
    """PID controller for balancing"""
    
    def __init__(self, kp, ki, kd, setpoint=0.0):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.setpoint = setpoint
        
        self.integral = 0.0
        self.last_error = 0.0
        self.last_time = time.monotonic()
    
    def update(self, current_value):
        """Calculate PID output"""
        current_time = time.monotonic()
        dt = current_time - self.last_time
        
        if dt <= 0.0:
            dt = 0.001  # Prevent division by zero
        
        # Calculate error
        error = self.setpoint - current_value
        
        # Proportional term
        p_term = self.kp * error
        
        # Integral term (with anti-windup)
        self.integral += error * dt
        # Limit integral to prevent windup
        max_integral = MAX_PWM / (2.0 * self.ki) if self.ki != 0 else 1000.0
        self.integral = max(-max_integral, min(max_integral, self.integral))
        i_term = self.ki * self.integral
        
        # Derivative term
        derivative = (error - self.last_error) / dt
        d_term = self.kd * derivative
        
        # Calculate output
        output = p_term + i_term + d_term
        
        # Update state
        self.last_error = error
        self.last_time = current_time
        
        if DEBUG:
            print(f"Error: {error:.2f}° | P: {p_term:.0f} I: {i_term:.0f} D: {d_term:.0f} | Out: {output:.0f}")
        
        return output
    
    def reset(self):
        """Reset PID state"""
        self.integral = 0.0
        self.last_error = 0.0
        self.last_time = time.monotonic()

# ============================================================================
# IMU ANGLE CALCULATION
# ============================================================================

class AngleEstimator:
    """Estimates tilt angle from IMU data"""
    
    def __init__(self, use_filter=True, alpha=0.98):
        self.use_filter = use_filter
        self.alpha = alpha
        self.angle = 0.0
        self.last_time = time.monotonic()

    def calculate_accel_angle(self, accel_x, accel_z):
        angle = math.atan2(accel_z, accel_x) * 180.0 / math.pi
        return angle
    
    def update(self, accel_x, accel_z, gyro_y):
        """
        Update angle estimate
        gyro_y is the rotation rate around Y-axis (tilting forward/back)
        """
        current_time = time.monotonic()
        dt = current_time - self.last_time
        
        if dt <= 0.0:
            dt = 0.001
        
        # Get angle from accelerometer
        accel_angle = self.calculate_accel_angle(accel_x, accel_z)
        
        if self.use_filter:
            # Complementary filter: combine gyro and accel
            # Integrate gyro for short-term accuracy
            gyro_rate = gyro_y * 180.0 / math.pi  # Convert rad/s to deg/s
            self.angle = self.alpha * (self.angle + gyro_rate * dt) + (1 - self.alpha) * accel_angle
        else:
            # Use only accelerometer
            self.angle = accel_angle
        
        self.last_time = current_time
        return self.angle

# ============================================================================
# MAIN PROGRAM
# ============================================================================

def main():
    print("=== Two-Wheeled Balancing Bot ===")
    print(f"PID: Kp={KP}, Ki={KI}, Kd={KD}")
    print(f"Filter: {'Complementary' if USE_COMPLEMENTARY_FILTER else 'Accel-only'}")
    print()
    
    # Initialize I2C and IMU
    i2c = board.I2C()
    
    try:
        icm = adafruit_icm20x.ICM20948(i2c, 0x69)
        print("ICM20948 found at address 0x69")
    except:
        print("Trying alternate address 0x68...")
        try:
            icm = adafruit_icm20x.ICM20948(i2c, 0x68)
            print("ICM20948 found at address 0x68")
        except:
            print("ERROR: No ICM20948 found!")
            return
    
    # Initialize motor driver
    # IMPORTANT: Update these pins to match your wiring!
    motors = DrokMotorDriver(
        in1_pin=board.D1,   # Motor 1 direction pin 1
        in2_pin=board.D2,  # Motor 1 direction pin 2
        ena1_pin=board.D6, # Motor 1 PWM pin
        in3_pin=board.D3,  # Motor 2 direction pin 1
        in4_pin=board.D4,  # Motor 2 direction pin 2
        ena2_pin=board.D7   # Motor 2 PWM pin
    )
    print("Motor driver initialized")
    
    # Initialize PID controller
    pid = PIDController(KP, KI, KD, setpoint=TARGET_ANGLE)
    
    # Initialize angle estimator
    angle_estimator = AngleEstimator(
        use_filter=USE_COMPLEMENTARY_FILTER,
        alpha=COMPLEMENTARY_ALPHA
    )
    
    print("Starting balance control in 2 seconds...")
    print("Tip the bot to near-vertical to begin!")
    time.sleep(2.0)
    
    # Main control loop
    try:
        while True:
            # Read IMU data
            accel_x, accel_y, accel_z = icm.acceleration
            gyro_x, gyro_y, gyro_z = icm.gyro
            
            # Calculate current angle
            current_angle = angle_estimator.update(accel_x, accel_z, gyro_y)
            
            # Check if bot has fallen over
            if abs(current_angle) > MAX_ANGLE:
                motors.brake()
                if DEBUG:
                    print(f"Angle too large: {current_angle:.1f}° - STOPPED")
                time.sleep(0.01)
                continue
            
            # Check if in deadband (close enough to balanced)
            if abs(current_angle - TARGET_ANGLE) < MOTOR_DEADBAND:
                pid_output = 0
            else:
                # Calculate PID output
                pid_output = pid.update(current_angle)
            
            # Apply minimum PWM threshold to overcome friction
            if abs(pid_output) < MIN_PWM and abs(pid_output) > 0:
                pid_output = MIN_PWM if pid_output > 0 else -MIN_PWM
            
            # Constrain output
            pid_output = max(-MAX_PWM, min(MAX_PWM, pid_output))
            
            # Drive motors
            # Positive PID output = leaning forward = need to drive forward
            motors.set_both_motors(int(pid_output))
            
            if DEBUG:
                print(f"Angle: {current_angle:6.2f}° | Motor: {int(pid_output):6d}")
            
            # Small delay for loop timing
            time.sleep(0.005)  # 200Hz update rate
    
    except KeyboardInterrupt:
        print("\nStopping...")
    finally:
        motors.brake()
        print("Motors stopped. Program ended.")

# ============================================================================
# RUN
# ============================================================================

if __name__ == "__main__":
    main()
