"""
Two-Wheeled Balancing Bot with Separate PID and BLE Tasks
This version demonstrates task coordination with shared state
"""
import asyncio
import time
import board
import busio
import pwmio
import digitalio
import adafruit_icm20x
import math
import analogio
import displayio
from adafruit_bitmap_font import bitmap_font
from adafruit_display_text import label
from adafruit_st7789 import ST7789
from fourwire import FourWire
import microcontroller
import neopixel
from audiocore import WaveFile
from audioio import AudioOut

# ============================================================================
# CONFIGURATION
# ============================================================================

DEBUG = True
USE_COMPLEMENTARY_FILTER = True

# PID Tuning Parameters
KP = 15000
KI = 0.0
KD = 10.0

# Motor constraints
MAX_PWM = 65535
MIN_PWM = 12000
MOTOR_DEADBAND = 0.25

# Target angle
TARGET_ANGLE = -2.3 

# Angle limits
MAX_ANGLE = 15.0

# Filter parameters
COMPLEMENTARY_ALPHA = 0.98

# BLE Configuration
BLE_BAUDRATE = 115200

# Battery Configuration
BATTERY_FULL_VOLTAGE = 8.4
BATTERY_EMPTY_VOLTAGE = 7.2
BATTERY_ADC_DIVIDER = 3.145 #Value measured with a multimeter # Voltage divider factor (R1+R2)/R2. R1=22k, R2=10k
BATTERY_UPDATE_INTERVAL = 100  # Seconds between battery/LCD updates
BATTERY_LOW_THRESHOLD = 10     # Percentage to trigger one-time alarm
BATTERY_CRITICAL_THRESHOLD = 5  # Percentage to trigger repeating alarm
BATTERY_ALARM_DURATION = 5     # Seconds to play alarm
BATTERY_ALARM_FILE = "/mixkit-alert-alarm-1005.wav"
ADC_REF = 3.3

# NeoPixel Configuration
NEOPIXEL_BRIGHTNESS = 0.1

# LCD Configuration
LCD_WIDTH = 135
LCD_HEIGHT = 240

# ============================================================================
# SHARED STATE CLASS
# ============================================================================

class SharedState:
    """Shared state between tasks"""
    def __init__(self):
        self.current_angle = 0.0
        self.pid_output = 0
        self.is_fallen = False
        self.update_count = 0
        self.lock = None  # Will be set in async context
    
    async def update_telemetry(self, angle, output, fallen=False):
        """Thread-safe update of telemetry data"""
        if self.lock:
            async with self.lock:
                self.current_angle = angle
                self.pid_output = output
                self.is_fallen = fallen
                self.update_count += 1
    
    async def get_telemetry(self):
        """Thread-safe read of telemetry data"""
        if self.lock:
            async with self.lock:
                return (self.current_angle, self.pid_output, 
                       self.is_fallen, self.update_count)
        return (self.current_angle, self.pid_output, 
               self.is_fallen, self.update_count)

# ============================================================================
# MOTOR CONTROL CLASS
# ============================================================================

class DrokMotorDriver:
    """Controls two DC motors via Drok driver board"""
    
    def __init__(self, in1_pin, in2_pin, ena1_pin, in3_pin, in4_pin, ena2_pin):
        self.in1 = digitalio.DigitalInOut(in1_pin)
        self.in1.direction = digitalio.Direction.OUTPUT
        self.in2 = digitalio.DigitalInOut(in2_pin)
        self.in2.direction = digitalio.Direction.OUTPUT
        self.ena1 = pwmio.PWMOut(ena1_pin, frequency=20000, duty_cycle=0)
        
        self.in3 = digitalio.DigitalInOut(in3_pin)
        self.in3.direction = digitalio.Direction.OUTPUT
        self.in4 = digitalio.DigitalInOut(in4_pin)
        self.in4.direction = digitalio.Direction.OUTPUT
        self.ena2 = pwmio.PWMOut(ena2_pin, frequency=20000, duty_cycle=0)
        
        self.motor1_trim = 1.0  
        self.motor2_trim = 1.0
        self.brake()
    
    def brake(self):
        self.in1.value = False
        self.in2.value = False
        self.in3.value = False
        self.in4.value = False
        self.ena1.duty_cycle = 0
        self.ena2.duty_cycle = 0
    
    def set_motor1(self, speed):
        if speed > 0:
            self.in1.value = True
            self.in2.value = False
            self.ena1.duty_cycle = min(int(abs(speed)), MAX_PWM)
        elif speed < 0:
            self.in1.value = False
            self.in2.value = True
            self.ena1.duty_cycle = min(int(abs(speed)), MAX_PWM)
        else:
            self.in1.value = False
            self.in2.value = False
            self.ena1.duty_cycle = 0
    
    def set_motor2(self, speed):
        if speed > 0:
            self.in3.value = True
            self.in4.value = False
            self.ena2.duty_cycle = min(int(abs(speed)), MAX_PWM)
        elif speed < 0:
            self.in3.value = False
            self.in4.value = True
            self.ena2.duty_cycle = min(int(abs(speed)), MAX_PWM)
        else:
            self.in3.value = False
            self.in4.value = False
            self.ena2.duty_cycle = 0
    
    def set_both_motors(self, speed):
        self.set_motor1(speed)
        self.set_motor2(speed)

# ============================================================================
# PID CONTROLLER CLASS
# ============================================================================

class PIDController:
    """PID controller with Slow Start gain ramping"""
    
    def __init__(self, kp, ki, kd, setpoint=0.0, ramp_time=2.0):
        self.target_kp = kp
        self.ki = ki
        self.target_kd = kd
        self.setpoint = setpoint
        self.ramp_time = ramp_time
        self.start_time = None
        self.integral = 0.0
        self.last_error = 0.0
        self.last_time = time.monotonic()

    def update(self, current_value):
        current_time = time.monotonic()
        if self.start_time is None:
            self.start_time = current_time
            
        dt = current_time - self.last_time
        if dt <= 0.0: dt = 0.001
        
        elapsed = current_time - self.start_time
        ramp_factor = min(1.0, elapsed / self.ramp_time)
        
        cur_kp = self.target_kp * ramp_factor
        cur_kd = self.target_kd * ramp_factor
        
        error = self.setpoint - current_value
        p_term = cur_kp * error
        
        self.integral += error * dt
        max_i = 65535 / (2.0 * self.ki) if self.ki != 0 else 1000.0
        self.integral = max(-max_i, min(max_i, self.integral))
        i_term = self.ki * self.integral
        
        derivative = (error - self.last_error) / dt
        d_term = cur_kd * derivative
        
        output = p_term + i_term + d_term
        
        self.last_error = error
        self.last_time = current_time
        
        if DEBUG and ramp_factor < 1.0:
            print(f"Slow Start: {int(ramp_factor*100)}% Power...")
            
        return output

    def reset(self):
        self.integral = 0.0
        self.last_error = 0.0
        self.last_time = time.monotonic()
        self.start_time = time.monotonic()

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
        current_time = time.monotonic()
        dt = current_time - self.last_time
        if dt <= 0.0:
            dt = 0.001
        
        accel_angle = self.calculate_accel_angle(accel_x, accel_z)
        
        if self.use_filter:
            gyro_rate = gyro_y * 180.0 / math.pi
            self.angle = self.alpha * (self.angle + gyro_rate * dt) + (1 - self.alpha) * accel_angle
        else:
            self.angle = accel_angle
        
        self.last_time = current_time
        return self.angle

# ============================================================================
# ASYNC TASKS
# ============================================================================

async def pid_control_task(motors, pid, angle_estimator, icm, shared_state):
    """
    PID control loop running at 200Hz
    Updates shared state with angle and PWM output
    """
    print("PID control task started")
    
    while True:
        loop_start = time.monotonic()
        
        # Read IMU
        accel_x, accel_y, accel_z = icm.acceleration
        gyro_x, gyro_y, gyro_z = icm.gyro
        
        # Calculate angle
        current_angle = angle_estimator.update(accel_x, accel_z, gyro_y)
        
        # Check if fallen
        if abs(current_angle) > MAX_ANGLE:
            motors.brake()
            pid.reset()
            await shared_state.update_telemetry(current_angle, 0, fallen=True)
            if DEBUG:
                print("Fallen! Resetting PID.")
            await asyncio.sleep(0.1)
            continue
        
        # Calculate PID
        if abs(current_angle - TARGET_ANGLE) < MOTOR_DEADBAND:
            pid_output = 0
        else:
            pid_output = pid.update(current_angle)
        
        # Apply constraints
        if abs(pid_output) < MIN_PWM and abs(pid_output) > 0:
            pid_output = MIN_PWM if pid_output > 0 else -MIN_PWM
        pid_output = max(-MAX_PWM, min(MAX_PWM, pid_output))
        
        # Drive motors
        motors.set_both_motors(int(pid_output))
        
        # Update shared state
        await shared_state.update_telemetry(current_angle, int(pid_output), fallen=False)
        
        if DEBUG:
            print(f"{current_angle:6.2f},{int(pid_output):6d}")
        
        # Maintain 200Hz
        elapsed = time.monotonic() - loop_start
        sleep_time = 0.005 - elapsed
        await asyncio.sleep(max(0, sleep_time))

async def ble_transmit_task(uart, shared_state):
    """
    BLE transmission task running at 200Hz
    Sends angle and PWM data from shared state
    """
    print("BLE transmit task started")
    
    last_update_count = 0
    last_send_time = 0
    send_interval = 0.05  # seconds (20Hz) - avoid overwhelming UART link

    while True:
        loop_start = time.monotonic()

        # Get telemetry from shared state
        angle, output, fallen, update_count = await shared_state.get_telemetry()

        # Throttle sends to avoid saturating the UART (and blocking)
        now = time.monotonic()
        if update_count != last_update_count and (now - last_send_time) >= send_interval:
            try:
                if fallen:
                    ble_msg = f"{angle:.2f},0,FALLEN\n"
                else:
                    ble_msg = f"{angle:.2f},{output}\n"
                # Guard write with try/except; if it raises, skip and try later
                uart.write(bytes(ble_msg, "ascii"))
                last_update_count = update_count
                last_send_time = now
            except Exception as e:
                if DEBUG:
                    print(f"BLE transmit error: {e}")

        # Run loop at ~20-200Hz, but avoid tight spinning
        elapsed = time.monotonic() - loop_start
        sleep_time = 0.01 - elapsed
        await asyncio.sleep(max(0, sleep_time))

async def ble_receive_task(uart):
    """
    BLE receive task for remote commands
    Runs at lower frequency (~20Hz)
    """
    print("BLE receive task started")

    has_in_waiting = hasattr(uart, "in_waiting")

    while True:
        try:
            if has_in_waiting:
                if uart.in_waiting > 0:
                    data = uart.read(uart.in_waiting)
                else:
                    data = None
            else:
                # Fallback: attempt a short non-blocking read if supported
                # (may still block on some builds; prefer in_waiting)
                data = uart.read(64)
        except Exception as e:
            if DEBUG:
                print(f"BLE receive error: {e}")
            data = None

        if data:
            try:
                print(data.decode("utf-8"), end="")
            except Exception:
                print(data)

        await asyncio.sleep(0.01)

async def play_alarm(audio, duration):
    """Play the alarm WAV file via DAC for the specified duration in seconds."""
    try:
        wave_file = open(BATTERY_ALARM_FILE, "rb")
        wave = WaveFile(wave_file)
        audio.play(wave, loop=True)
        await asyncio.sleep(duration)
        audio.stop()
        wave_file.close()
    except Exception as e:
        if DEBUG:
            print(f"Alarm playback error: {e}")

async def battery_display_task(battery_adc, uart, text_battery_pct, text_battery_volt, pixels, audio):
    """
    Reads battery voltage, updates LCD display and NeoPixel 0,
    and sends battery stats via BLE every BATTERY_UPDATE_INTERVAL seconds.
    Plays alarm when battery is low.
    """
    print("Battery display task started")

    low_alarm_played = False
    last_critical_alarm = 0

    while True:
        # Read battery voltage from ADC
        adc_value = battery_adc.value
        print("ADC Val: ", adc_value)
        voltage = (adc_value / 65535) * ADC_REF * BATTERY_ADC_DIVIDER
        print("Battery Voltage: ", voltage)

        # Calculate battery percentage (linear mapping between empty and full)
        pct = (voltage - BATTERY_EMPTY_VOLTAGE) / (BATTERY_FULL_VOLTAGE - BATTERY_EMPTY_VOLTAGE) * 100.0
        pct = max(0.0, min(100.0, pct))

        # Determine color based on battery percentage
        if pct > 50:
            text_color = 0x00FF00  # Green
            pixel_color = (0, 255, 0)
        elif pct >= 25:
            text_color = 0xFF8000  # Orange
            pixel_color = (255, 128, 0)
        else:
            text_color = 0xFF0000  # Red
            pixel_color = (255, 0, 0)

        # Update LCD text and color
        text_battery_pct.text = f"{pct:.0f}%"
        text_battery_pct.color = text_color
        text_battery_volt.text = f"{voltage:.2f}V"
        text_battery_volt.color = text_color

        # Update NeoPixel 0
        pixels[0] = pixel_color
        pixels.show()

        # Send battery stats via BLE
        try:
            ble_msg = f"BAT:{pct:.0f}%,{voltage:.2f}V\n"
            uart.write(bytes(ble_msg, "ascii"))
        except Exception:
            pass

        if DEBUG:
            print(f"Battery: {pct:.0f}% ({voltage:.2f}V)")

        # Low battery alarm: one-time when first dropping below 20%
        if pct < BATTERY_LOW_THRESHOLD and not low_alarm_played:
            low_alarm_played = True
            print("Low battery alarm!")
            await play_alarm(audio, BATTERY_ALARM_DURATION)

        # Critical battery alarm: every 60 seconds when below 10%
        if pct < BATTERY_CRITICAL_THRESHOLD:
            now = time.monotonic()
            if now - last_critical_alarm >= 60:
                last_critical_alarm = now
                print("Critical battery alarm!")
                await play_alarm(audio, BATTERY_ALARM_DURATION)

        await asyncio.sleep(BATTERY_UPDATE_INTERVAL)

# ============================================================================
# MAIN PROGRAM
# ============================================================================

async def main():
    print("=== Balancing Bot with Separate PID and BLE Tasks ===")
    print(f"PID: Kp={KP}, Ki={KI}, Kd={KD}")
    print(f"BLE: {BLE_BAUDRATE} baud")
    print()
    
    # Initialize I2C and IMU
    i2c = board.I2C()
    try:
        icm = adafruit_icm20x.ICM20948(i2c, 0x69)
        print("ICM20948 found at 0x69")
    except:
        try:
            icm = adafruit_icm20x.ICM20948(i2c, 0x68)
            print("ICM20948 found at 0x68")
        except:
            print("ERROR: No ICM20948 found!")
            return
    
    # Initialize BLE UART (use BLE_TX/BLE_RX when available, fall back to TX/RX)
    try:
        tx = getattr(board, "BLE_TX", getattr(board, "TX", None))
        rx = getattr(board, "BLE_RX", getattr(board, "RX", None))
        if tx is None or rx is None:
            raise RuntimeError("No UART TX/RX pins available on this board - update pin names.")
        # Use a short timeout so `read()` is effectively non-blocking
        uart = busio.UART(tx, rx, baudrate=BLE_BAUDRATE, timeout=0.01)
        print(f"BLE UART initialized at {BLE_BAUDRATE} baud (tx={tx}, rx={rx})")
    except Exception as e:
        print(f"ERROR: BLE UART failed: {e}")
        return
    
    # Initialize battery ADC
    battery_adc = analogio.AnalogIn(board.A0)
    print("Battery ADC initialized")

    # Initialize audio output via DAC (WAV files must be Mono 16-bit at 22kHz or less)
    audio = AudioOut(board.DAC)
    print("Audio DAC initialized")

    # Initialize NeoPixels
    pixels = neopixel.NeoPixel(board.NEOPIXEL, 5, brightness=NEOPIXEL_BRIGHTNESS, auto_write=False)
    pixels.fill(0x000000)
    pixels.show()
    print("NeoPixels initialized")

    # Initialize LCD display
    displayio.release_displays()
    spi = board.LCD_SPI()
    tft_cs = board.LCD_CS
    tft_dc = board.D4
    backlight = digitalio.DigitalInOut(microcontroller.pin.PA06)
    backlight.direction = digitalio.Direction.OUTPUT
    backlight.value = False  # Turn backlight on

    display_bus = FourWire(spi, command=tft_dc, chip_select=tft_cs)
    display = ST7789(
        display_bus, rotation=0, width=LCD_WIDTH, height=LCD_HEIGHT, rowstart=40, colstart=53
    )

    font = bitmap_font.load_font("/Roboto-Regular-47.bdf")
    text_battery_pct = label.Label(font, text="---%", color=0x00FF00)
    text_battery_pct.x = 0
    text_battery_pct.y = 40

    text_battery_volt = label.Label(font, text="-.-V", color=0x00FF00)
    text_battery_volt.x = 0
    text_battery_volt.y = 120

    parent_group = displayio.Group()
    parent_group.append(text_battery_pct)
    parent_group.append(text_battery_volt)
    display.root_group = parent_group
    print("LCD display initialized")

    # Initialize motors
    motors = DrokMotorDriver(
        in1_pin=board.D2, in2_pin=board.D3, ena1_pin=board.D1,
        in3_pin=board.D5, in4_pin=board.D6, ena2_pin=board.D7
    )
    motors.motor2_trim = 1.6
    print("Motor driver initialized")
    
    # Initialize PID and angle estimator
    pid = PIDController(KP, KI, KD, setpoint=TARGET_ANGLE, ramp_time=3.0)
    angle_estimator = AngleEstimator(
        use_filter=USE_COMPLEMENTARY_FILTER,
        alpha=COMPLEMENTARY_ALPHA
    )
    
    # Create shared state with asyncio lock
    shared_state = SharedState()
    shared_state.lock = asyncio.Lock()
    
    print("Starting in 2 seconds...")
    await asyncio.sleep(2.0)
    
    # Create and run tasks
    tasks = [
        asyncio.create_task(pid_control_task(motors, pid, angle_estimator, icm, shared_state)),
        asyncio.create_task(ble_transmit_task(uart, shared_state)),
        asyncio.create_task(battery_display_task(battery_adc, uart, text_battery_pct, text_battery_volt, pixels, audio)),
        asyncio.create_task(ble_receive_task(uart)),  # Optional
    ]
    
    try:
        await asyncio.gather(*tasks)
    except KeyboardInterrupt:
        print("\nStopping...")
    finally:
        motors.brake()
        print("Motors stopped.")

# ============================================================================
# RUN
# ============================================================================

if __name__ == "__main__":
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        print("\nProgram terminated")
