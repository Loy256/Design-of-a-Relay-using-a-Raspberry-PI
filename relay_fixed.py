#!/usr/bin/env python3
import numpy as np
import time
#import RPi.GPIO as GPIO
import board
import busio
import adafruit_ads1x15.ads1115 as ADS
from adafruit_ads1x15.analog_in import AnalogIn
from RPLCD.i2c import CharLCD
import signal
import sys

# =========================
# HARDWARE SETUP
# =========================
# =========================
# HARDWARE SETUP
# =========================
# GPIO
from gpiozero import OutputDevice
import lgpio

TRIP_PIN = None

def init_gpio():
    """Initialize GPIO with error handling"""
    global TRIP_PIN
    try:
        TRIP_PIN = OutputDevice(11, active_high=True, initial_value=False)
        print("✓ GPIO11 initialized successfully")
        return True
    except Exception as e:
        print(f"⚠ Warning: GPIO11 initialization failed: {e}")
        print("  Attempting to reset GPIO...")
        try:
            # Try to reset the GPIO
            import os
            os.system("gpio -g mode 11 in")
            os.system("gpio -g mode 11 out")
            time.sleep(0.5)
            TRIP_PIN = OutputDevice(11, active_high=True, initial_value=False)
            print("✓ GPIO11 reset and initialized successfully")
            return True
        except Exception as e2:
            print(f"✗ GPIO11 initialization still failed: {e2}")
            print("  Relay will work but trip command will be disabled")
            return False

# Graceful shutdown handler
def cleanup(signum=None, frame=None):
    """Clean up resources on exit"""
    print("\n\nShutting down...")
    if TRIP_PIN is not None:
        try:
            TRIP_PIN.off()
            TRIP_PIN.close()
            print("✓ GPIO cleaned up")
        except:
            pass
    sys.exit(0)

# Register signal handlers for graceful shutdown
signal.signal(signal.SIGINT, cleanup)
signal.signal(signal.SIGTERM, cleanup)

# I2C bus
i2c = busio.I2C(board.SCL, board.SDA)

# ADS1115 ADC
ads = ADS.ADS1115(i2c)
# =========================
# ADS1115 ADC
# =========================
#ads = ADS.ADS1115(i2c)

# Gain 1 = ±4.096 V range
ads.gain = 1

# More stable than 860 SPS
ads.data_rate = 860

CH_CURRENT = AnalogIn(ads, 0)
CH_VOLTAGE = AnalogIn(ads, 1)

# =========================
# I2C DEVICE SCANNING
# =========================
def scan_i2c_devices(i2c_bus):
    """Scan I2C bus and return list of detected device addresses"""
    devices = []
    for address in range(0x03, 0x78):
        try:
            i2c_bus.writeto(address, bytes([0]))
            devices.append(hex(address))
            print(f"I2C device found at address: {hex(address)}")
        except (OSError, IOError):
            pass
    return devices

# Scan for connected devices (for debugging)
print("Scanning I2C bus for devices...")
detected_devices = scan_i2c_devices(i2c)
print(f"Detected I2C devices: {detected_devices}")

# =========================
# LCD 16x4 - Auto-detect address
# =========================
lcd = None
lcd_address = None

# First try common LCD expander addresses
common_addresses = [0x27, 0x3F, 0x20, 0x21,0x22, 0x23, 0x24, 0x25, 0x26, 0x38]

# Then try all detected addresses that aren't the ADS1115 (0x48)
detected_addrs = [int(addr, 16) for addr in detected_devices if addr != '0x48']
addresses_to_try = list(dict.fromkeys(common_addresses + detected_addrs))  # Remove duplicates

print(f"\nTrying to initialize LCD at detected addresses: {[hex(a) for a in addresses_to_try]}")

for addr in addresses_to_try:
    try:
        print(f"Attempting to initialize LCD at address {hex(addr)}...")
        lcd = CharLCD(
            i2c_expander='PCF8574',
            address=addr,
            port=1,
            cols=16,
            rows=4,
            auto_linebreaks=False
        )
        lcd.clear()
        lcd_address = addr
        print(f"✓ Successfully initialized LCD at address {hex(addr)}")
        break
    except (OSError, IOError) as e:
        print(f"  LCD not found at {hex(addr)}")
        lcd = None

if lcd is None:
    print("\n⚠ Warning: LCD could not be initialized at any address")
    print("Your LCD may not be connected or powered on.")
    print("Continuing without LCD display...")
    print("Expected values on LCD:")
    print("  - Line 1: 'Relay READY' or 'FAULT DETECTED' or 'NORMAL' or 'TRIP'")
    print("  - Line 2: Current (I) and Voltage (V) readings")
    print("  - Line 3: Mode info or 'Monitoring'")
    print("  - Line 4: Status info")

# =========================
# CONFIGURATION
# =========================
SYSTEM_FREQUENCY = 50
SAMPLES_PER_CYCLE = 8         # FIX: increased from 10 for better FFT
SETTING_CURRENT_RMS = 1.0
TMS = 0.3
RELAY_CHAR_ANGLE = 0           # FIX: added missing parameter
RELAY_MODE = "IDMT"
FORWARD_HALF_ANGLE = 90

# =========================
# VIRTUAL DISC (for IDMT)
# =========================
class VirtualDisc:
    """Virtual disc for IDMT timing"""
    def __init__(self):
        self.position = 0.0
        self.HALF_CYCLE_DURATION = 1.0 / (2 * SYSTEM_FREQUENCY)
    
    def reset(self):
        self.position = 0.0
    
    def idmt_trip_time(self, i_rms: float) -> float:
        """Standard Inverse trip time"""
        ratio = i_rms / SETTING_CURRENT_RMS
        if ratio <= 1.0:
            return float("inf")
        return (TMS * 0.14) / (ratio ** 0.02 - 1)
    
    def advance(self, i_rms: float) -> bool:
        """Advance disc by one cycle, return True when ready to trip"""
        t_trip = self.idmt_trip_time(i_rms)
        if t_trip == float("inf"):
            self.position = max(0.0, self.position - 5.0)
            return False
        
        interval = 1.0 / (SYSTEM_FREQUENCY * SAMPLES_PER_CYCLE)
        increment = (interval / t_trip) * 100.0
        self.position = min(100.0, self.position + increment)
        return self.position >= 100.0

disc = VirtualDisc()

# =========================
# ADC READING
# =========================
def read_voltage(channel):
    v = channel.voltage

    # Prevent impossible values
    if v < 0:
        v = 0

    if v > 4.096:
        v = 4.096

    return v

# =========================
# SAMPLING (FIXED for better timing)
# =========================
def sample_cycle():
    i_samples = []
    v_samples = []

    interval = 1.0 / (SYSTEM_FREQUENCY * SAMPLES_PER_CYCLE)
    next_time = time.perf_counter()

    for _ in range(SAMPLES_PER_CYCLE):
        i_samples.append(read_voltage(CH_CURRENT))
        v_samples.append(read_voltage(CH_VOLTAGE))

        next_time += interval

        while time.perf_counter() < next_time:
            time.sleep(0.00005)

    return np.array(i_samples), np.array(v_samples)

# =========================
# FFT (stable version)
# =========================
def fft_rms(x):
    if len(x) == 0:
        return 0.0

    x = x - np.mean(x)

    if np.max(np.abs(x)) < 0.01:
        return 0.0

    x = x * np.hanning(len(x))

    X = np.fft.fft(x)

    fundamental = X[1]

    mag = (2.0 / len(x)) * abs(fundamental)

    return mag / np.sqrt(2)


# =========================
# FFT PHASE DETECTION
# =========================
def fft_phase(x):
    """
    Returns phase angle of the fundamental frequency component.
    """
    if len(x) == 0:
        return 0.0

    x = x - np.mean(x)

    if np.max(np.abs(x)) < 0.01:
        return 0.0

    x = x * np.hanning(len(x))

    X = np.fft.fft(x)

    fundamental = X[1]

    phase = np.angle(fundamental, deg=True)

    return phase


# =========================
# LCD FUNCTION (FIXED)
# =========================
def lcd_print(l1="", l2="", l3="", l4=""):
    if lcd is None:
        # Print to console if LCD not available
        print(f"LCD: {l1} | {l2} | {l3} | {l4}")
        return
    
    lcd.clear()

    lcd.cursor_pos = (0, 0)
    lcd.write_string(str(l1)[:16])

    lcd.cursor_pos = (1, 0)
    lcd.write_string(str(l2)[:16])

    lcd.cursor_pos = (2, 0)
    lcd.write_string(str(l3)[:16])

    lcd.cursor_pos = (3, 0)
    lcd.write_string(str(l4)[:16])

# =========================
# FAULT DIRECTION
# =========================
def is_forward(i, v, pre_fault_v_angle=None):
    """Determine if fault is forward direction"""
    v_rms = fft_rms(v)
    
    # FIX: Handle voltage collapse
    if v_rms < 0.05 and pre_fault_v_angle is not None:
        v_phase = pre_fault_v_angle
    else:
        v_phase = fft_phase(v)
    
    i_phase = fft_phase(i)
    
    # Phase difference
    phase_diff = i_phase - v_phase
    phase_diff = (phase_diff + 180) % 360 - 180
    
    # Adjust by RCA
    adjusted = phase_diff - RELAY_CHAR_ANGLE
    adjusted = (adjusted + 180) % 360 - 180
    
    forward = abs(adjusted) <= FORWARD_HALF_ANGLE
    return forward

# =========================
# TRIP
# =========================
def trip():
    if TRIP_PIN is not None:
        TRIP_PIN.on()
        time.sleep(0.1)
        TRIP_PIN.off()
        print("⚡ RELAY TRIPPED")
    else:
        print("⚠ Trip signal would be sent (GPIO not available)")

# =========================
# MAIN LOOP
# =========================

def run():
    init_gpio()
    last_update = 0
    LCD_RATE = 0.2
    pre_fault_v_angle = None
    lcd_print("Relay READY", "ADS1115 ACTIVE", "", "")

    while True:
        i, v = sample_cycle()

        print(
            f"RAW_CURRENT={CH_CURRENT.voltage:.4f}V "
            f"RAW_VOLTAGE={CH_VOLTAGE.voltage:.4f}V"
        )

        i_rms = fft_rms(i)
        v_rms = fft_rms(v)

        fault = i_rms > SETTING_CURRENT_RMS

        if time.time() - last_update > LCD_RATE:
            if fault:
                if RELAY_MODE == "INSTANTANEOUS":
                    lcd_print(
                        "FAULT DETECTED",
                        f"I={i_rms:.2f}A V={v_rms:.1f}V",
                        f"Mode={RELAY_MODE}",
                        "Checking dir..."
                    )
                else:  # IDMT
                    lcd_print(
                        f"IDMT {disc.position:.0f}%",
                        f"I={i_rms:.2f}A V={v_rms:.1f}V",
                        f"t={disc.idmt_trip_time(i_rms):.2f}s",
                        "Accumulating..."
                    )
            else:
                lcd_print(
                    "NORMAL",
                    f"I={i_rms:.2f}A V={v_rms:.1f}V",
                    "Monitoring",
                    ""
                )
            last_update = time.time()

        if fault:
            if RELAY_MODE == "INSTANTANEOUS":
                if is_forward(i, v, pre_fault_v_angle):
                    lcd_print("TRIP", "FORWARD FAULT", "BREAKER OPEN", "")
                    trip()
                    time.sleep(1)
            else:  # IDMT mode
                if disc.advance(i_rms):
                    if is_forward(i, v, pre_fault_v_angle):
                        lcd_print("TRIP", "FORWARD FAULT", "BREAKER OPEN", "")
                        trip()
                        disc.reset()
                        time.sleep(1)
                    else:
                        disc.reset()
        else:
            pre_fault_v_angle = fft_phase(v)
            disc.reset()
# =========================
# ENTRY
# =========================
if __name__ == "__main__":
    try:
        run()
    except KeyboardInterrupt:
        cleanup()
    except Exception as e:
        print(f"Error: {e}")
        cleanup()
