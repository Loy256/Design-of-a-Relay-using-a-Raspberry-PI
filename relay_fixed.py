#!/usr/bin/env python3
import numpy as np
import time
#import RPi.GPIO as GPIO
import board
import busio
import adafruit_ads1x15.ads1115 as ADS
from adafruit_ads1x15.analog_in import AnalogIn
from RPLCD.i2c import CharLCD

# =========================
# HARDWARE SETUP
# =========================
# GPIO
from gpiozero import OutputDevice

TRIP_PIN = OutputDevice(17, active_high=True, initial_value=False)
# I2C bus
i2c = busio.I2C(board.SCL, board.SDA)

# ADS1115 ADC
ads = ADS.ADS1115(i2c)
# =========================
# ADS1115 ADC
# =========================
ads = ADS.ADS1115(i2c)

# Gain 1 = ±4.096 V range
ads.gain = 1

# More stable than 860 SPS
ads.data_rate = 475

CH_CURRENT = AnalogIn(ads, ADS.P0)
CH_VOLTAGE = AnalogIn(ads, ADS.P1)

# =========================
# LCD 16x4
# =========================
lcd = CharLCD(
    i2c_expander='PCF8574',
    address=0x3F,
    port=1,
    cols=16,
    rows=4,
    auto_linebreaks=False
)
lcd.clear()

# =========================
# CONFIGURATION
# =========================
SYSTEM_FREQUENCY = 50
SAMPLES_PER_CYCLE = 32         # FIX: increased from 10 for better FFT
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
    TRIP_PIN.on()
    time.sleep(0.1)
    TRIP_PIN.off()

# =========================
# MAIN LOOP
# =========================
def run():
    last_update = 0
    LCD_RATE = 0.2
    pre_fault_v_angle = None  # FIX: store reference voltage angle
    
    lcd_print("Relay READY", "ADS1115 ACTIVE", "", "")
    
    while True:
        i, v = sample_cycle()
        i_rms = fft_rms(i)
        v_rms = fft_rms(v)
        fault = i_rms > SETTING_CURRENT_RMS
        
        if time.time() - last_update > LCD_RATE:
            if fault:
                # FIX: Show all 4 lines with more info
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
            # FIX: Implement IDMT mode properly
            if RELAY_MODE == "INSTANTANEOUS":
                if is_forward(i, v, pre_fault_v_angle):
                    lcd_print("TRIP", "FORWARD FAULT", "BREAKER OPEN", "")
                    trip()
                    time.sleep(1)
            else:  # IDMT mode
                if disc.advance(i_rms):  # Disc reached 100%
                    if is_forward(i, v, pre_fault_v_angle):
                        lcd_print("TRIP", "FORWARD FAULT", "BREAKER OPEN", "")
                        trip()
                        disc.reset()
                        time.sleep(1)
                    else:
                        # FIX: Reset disc on reverse fault
                        disc.reset()
        else:
            # Below pickup: store pre-fault angle and reset disc
            pre_fault_v_angle = fft_phase(v)
            disc.reset()

# =========================
# ENTRY
# =========================
if __name__ == "__main__":
    try:
        run()
    except KeyboardInterrupt:
        pass
    #TRIP_PIN.close()