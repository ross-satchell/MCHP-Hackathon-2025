"""
Two-Wheeled Balancing Bot with PID Control and BLE Telemetry
Uses ICM20948 IMU, Drok dual motor driver, and asyncio for cooperative multitasking
"""
import asyncio
import time
import board
import busio
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
KP = 18000    # Proportional gain - how aggressively to respond to current erro
KI = 30.0      # Integral gain - eliminates steady-state error
KD = 220.0     # Derivative gain - dampens oscillations
d_filter_alpha = 0.7

# Motor constraints
MAX_PWM = 65535      # Maximum PWM value (16-bit)
MIN_PWM = 12000       # Minimum PWM to overcome motor friction
MOTOR_DEADBAND = 0.15 # Angle deadband (degrees) where motors stay off

# Target angle (degrees from vertical)
TARGET_ANGLE = -2.3

# Angle limits for safety
MAX_ANGLE = 20.0     # If tilt exceeds this, stop trying (fallen over)

# Filter parameters
COMPLEMENTARY_ALPHA = 0.99  # Weight for gyro vs accel (0.98 = 98% gyro, 2% accel)

# Task Frequencies
PID_FREQ_HZ  = 200   # PID control loop frequency (Hz)
BLE_FREQ_HZ  = 200   # BLE transmit frequency (Hz) — can differ from PID_FREQ_HZ

PID_PERIOD   = 1.0 / PID_FREQ_HZ   # seconds per PID loop iteration
BLE_PERIOD   = 1.0 / BLE_FREQ_HZ   # seconds per BLE transmit iteration

# BLE Configuration
BLE_BAUDRATE = 115200  # Match your BLE module

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

        # Balance factor: 1.0 means 100% speed. 
        # Reduce the stronger motor (e.g., 0.9) to match the weaker one.
        self.motor1_trim = 1.0  
        self.motor2_trim = 1.0
        
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

"""
IMPROVED PID Controller with Derivative Filtering
Replace the PIDController class in your code with this version
"""

class PIDController:
    """PID controller with Slow Start gain ramping AND derivative filtering"""
    
    def __init__(self, kp, ki, kd, setpoint=0.0, ramp_time=2.0, d_filter_alpha=0.5):
        self.target_kp = kp
        self.ki = ki
        self.target_kd = kd
        self.setpoint = setpoint
        
        self.ramp_time = ramp_time
        self.start_time = None  # Will be set when ramping begins
        
        self.integral = 0.0
        self.last_error = 0.0
        self.last_time = time.monotonic()
        
        # NEW: Derivative filtering to reduce noise amplification
        self.d_filter_alpha = d_filter_alpha  # 0.0 = no filtering, 1.0 = max filtering
        self.filtered_derivative = 0.0

    def update(self, current_value):
        """Calculate PID output with ramped gains and filtered derivative"""
        current_time = time.monotonic()
        
        # Initialize start_time on the first update call
        if self.start_time is None:
            self.start_time = current_time
            
        dt = current_time - self.last_time
        if dt <= 0.0: dt = 0.001
        
        # Calculate Ramp Factor (0.0 to 1.0)
        elapsed = current_time - self.start_time
        ramp_factor = min(1.0, elapsed / self.ramp_time)
        
        # Current Ramped Gains
        cur_kp = self.target_kp * ramp_factor
        cur_kd = self.target_kd * ramp_factor
        
        # Calculate error
        error = self.setpoint - current_value
        
        # Proportional term
        p_term = cur_kp * error
        
        # Integral term (with anti-windup)
        self.integral += error * dt
        max_i = 65535 / (2.0 * self.ki) if self.ki != 0 else 1000.0
        self.integral = max(-max_i, min(max_i, self.integral))
        i_term = self.ki * self.integral
        
        # Derivative term WITH FILTERING
        # Calculate raw derivative
        raw_derivative = (error - self.last_error) / dt
        
        # Apply exponential moving average filter to smooth out noise
        # Higher alpha = more filtering (slower response, less noise)
        # Lower alpha = less filtering (faster response, more noise)
        self.filtered_derivative = (self.d_filter_alpha * self.filtered_derivative + 
                                   (1 - self.d_filter_alpha) * raw_derivative)
        
        d_term = cur_kd * self.filtered_derivative
        
        # Calculate output
        output = p_term + i_term + d_term
        
        # Update state
        self.last_error = error
        self.last_time = current_time
        
        if DEBUG and ramp_factor < 1.0:
            print(f"Slow Start: {int(ramp_factor*100)}% Power...")
            
        return output

    def reset(self):
        """Reset PID state and restart the ramp"""
        self.integral = 0.0
        self.last_error = 0.0
        self.last_time = time.monotonic()
        self.start_time = time.monotonic()  # Re-trigger slow start
        self.filtered_derivative = 0.0  # Reset derivative filter


# Example of how to initialize with derivative filtering:
# In your main() function, replace:
#   pid = PIDController(KP, KI, KD, setpoint=TARGET_ANGLE, ramp_time=3.0)
# With:
#   pid = PIDController(KP, KI, KD, setpoint=TARGET_ANGLE, ramp_time=3.0, d_filter_alpha=0.7)
#
# d_filter_alpha recommendations:
#   0.5 = moderate filtering (good starting point)
#   0.7 = heavy filtering (for very noisy systems)
#   0.3 = light filtering (for smooth, low-noise systems)


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
# ASYNC TASKS
# ============================================================================

async def balance_control_task(motors, pid, angle_estimator, icm, uart):
    """
    Main balance control loop running at PID_FREQ_HZ.
    Sends telemetry via BLE after each PID calculation at BLE_FREQ_HZ.
    Both periods are derived from their respective frequency constants.
    """
    print(f"Balance control task started  (PID:{PID_FREQ_HZ}Hz  BLE:{BLE_FREQ_HZ}Hz)")
    
    loop_count = 0
    
    while True:
        loop_start = time.monotonic()
        
        # Read IMU data
        accel_x, accel_y, accel_z = icm.acceleration
        gyro_x, gyro_y, gyro_z = icm.gyro
        
        # Calculate current angle
        current_angle = angle_estimator.update(accel_x, accel_z, gyro_y)
        
        # Check if bot has fallen over
        if abs(current_angle) > MAX_ANGLE:
            motors.brake()
            pid.reset()
            if DEBUG:
                print("Fallen! Resetting PID and Gain Ramp.")
            
            # Send fallen status via BLE
            try:
                ble_msg = f"{current_angle:.2f},0,FALLEN\n"
                uart.write(bytes(ble_msg, "ascii"))
            except:
                pass
            
            await asyncio.sleep(10 * PID_PERIOD)  # ~10 loop periods recovery pause
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
        motors.set_both_motors(int(pid_output))
        
        # Send telemetry via BLE after PID calculation
        try:
            ble_msg = f"{current_angle:.2f},{int(pid_output)}\n"
            uart.write(bytes(ble_msg, "ascii"))
        except Exception as e:
            # Continue even if BLE write fails
            if DEBUG and loop_count % 200 == 0:  # Print error occasionally
                print(f"BLE write error: {e}")
        
        # Debug output to console
        if DEBUG:
            print(f"{current_angle:6.2f},{int(pid_output):6d}")
        
        # Maintain PID_FREQ_HZ update rate — sleep for remainder of period
        elapsed = time.monotonic() - loop_start
        sleep_time = PID_PERIOD - elapsed

        if sleep_time > 0:
            await asyncio.sleep(sleep_time)
        else:
            # Work exceeded the period — yield briefly so other tasks can run
            await asyncio.sleep(0)
        
        loop_count += 1

async def ble_receive_task(uart):
    """
    Optional task to receive commands via BLE
    Runs independently and can be used for parameter tuning
    """
    print("BLE receive task started")
    
    while True:
        # Check for incoming BLE data
        try:
            data = uart.read(32)
            if data is not None:
                data_string = ''.join([chr(b) for b in data])
                print(f"BLE RX: {data_string}")
                # Here you could parse commands like "KP=7000" to adjust parameters
        except:
            pass
        
        # Run at BLE_FREQ_HZ
        await asyncio.sleep(BLE_PERIOD)

# ============================================================================
# MAIN PROGRAM
# ============================================================================

async def main():
    print("=== Two-Wheeled Balancing Bot with BLE ===")
    print(f"PID: Kp={KP}, Ki={KI}, Kd={KD}")
    print(f"Filter: {'Complementary' if USE_COMPLEMENTARY_FILTER else 'Accel-only'}")
    print(f"PID loop: {PID_FREQ_HZ}Hz  (period {PID_PERIOD*1000:.2f}ms)")
    print(f"BLE tx:   {BLE_FREQ_HZ}Hz  (period {BLE_PERIOD*1000:.2f}ms)")
    print(f"Target Angle: {TARGET_ANGLE}°")
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
    
    # Initialize BLE UART
    try:
        uart = busio.UART(board.BLE_TX, board.BLE_RX, baudrate=BLE_BAUDRATE)
        print(f"BLE UART initialized at {BLE_BAUDRATE} baud")
    except Exception as e:
        print(f"WARNING: BLE UART initialization failed: {e}")
        print("Continuing without BLE...")
        uart = None
    
    # Initialize motor driver
    motors = DrokMotorDriver(
        in1_pin=board.D2,   # Motor 1 direction pin 1
        in2_pin=board.D3,   # Motor 1 direction pin 2
        ena1_pin=board.D4,  # Motor 1 PWM pin
        in3_pin=board.D5,   # Motor 2 direction pin 1
        in4_pin=board.D6,   # Motor 2 direction pin 2
        ena2_pin=board.D7   # Motor 2 PWM pin
    )
    motors.motor2_trim = 1.6
    print("Motor driver initialized")
    
    # Initialize PID controller with 3 second ramp
    pid = PIDController(KP, KI, KD, setpoint=TARGET_ANGLE, ramp_time=3.0, d_filter_alpha=0.7)
    
    # Initialize angle estimator
    angle_estimator = AngleEstimator(
        use_filter=USE_COMPLEMENTARY_FILTER,
        alpha=COMPLEMENTARY_ALPHA
    )
    
    print("Starting balance control in 2 seconds...")
    print("Tip the bot to near-vertical to begin!")
    await asyncio.sleep(2.0)
    
    # Create tasks
    tasks = []
    
    # Balance control task (required)
    if uart:
        tasks.append(asyncio.create_task(
            balance_control_task(motors, pid, angle_estimator, icm, uart)
        ))
    else:
        print("ERROR: Cannot run without BLE UART")
        return
    
    # Optional: BLE receive task for remote commands
    # Uncomment to enable receiving commands via BLE
    # tasks.append(asyncio.create_task(ble_receive_task(uart)))
    
    # Run all tasks
    try:
        await asyncio.gather(*tasks)
    except KeyboardInterrupt:
        print("\nStopping...")
    finally:
        motors.brake()
        print("Motors stopped. Program ended.")

# ============================================================================
# RUN
# ============================================================================

if __name__ == "__main__":
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        print("\nProgram terminated by user")
