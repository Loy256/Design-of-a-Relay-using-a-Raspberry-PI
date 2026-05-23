#!/usr/bin/env python3

import numpy as np
import time
import RPi.GPIO as GPIO

import board
import busio
import adafruit_ads1x15.ads1115 as ADS
from adafruit_ads1x15.analog_in import AnalogIn

from RPLCD.i2c import CharLCD


# =========================
# HARDWARE SETUP
# =========================

# GPIO
TRIP_PIN = 17
GPIO.setmode(GPIO.BCM)
GPIO.setup(TRIP_PIN, GPIO.OUT, initial=GPIO.LOW)

# I2C bus
i2c = busio.I2C(board.SCL, board.SDA)

# ADS1115 ADC
ads = ADS.ADS1115(i2c)
ads.gain = 1          # ±4.096V range (good for sensors)
ads.data_rate = 475   # max stable rate

CH_CURRENT = AnalogIn(ads, ADS.P0)
CH_VOLTAGE = AnalogIn(ads, ADS.P1)

# LCD 16x4
lcd = CharLCD(
    i2c_expander='PCF8574',
    address=0x27,
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
SAMPLES_PER_CYCLE = 10   # adjusted for ADS1115 reality

SETTING_CURRENT_RMS = 1.0
TMS = 0.3
RELAY_MODE = "IDMT"

FORWARD_HALF_ANGLE = 90


# =========================
# ADC READING
# =========================

def read_voltage(channel: AnalogIn):
    # ADS1115 already returns voltage correctly scaled
    return channel.voltage


# =========================
# SAMPLING
# =========================

def sample_cycle():
    i_samples = []
    v_samples = []

    interval = 1.0 / (SYSTEM_FREQUENCY * SAMPLES_PER_CYCLE)

    for _ in range(SAMPLES_PER_CYCLE):
        i_samples.append(read_voltage(CH_CURRENT))
        v_samples.append(read_voltage(CH_VOLTAGE))
        time.sleep(interval)

    return np.array(i_samples), np.array(v_samples)


# =========================
# FFT (stable version)
# =========================

def fft_rms(x):
    x = x * np.hanning(len(x))
    X = np.fft.fft(x)
    mag = np.abs(X[1]) * 2 / len(x)
    return mag / np.sqrt(2)


def fft_phase(x):
    X = np.fft.fft(x * np.hanning(len(x)))
    return np.angle(X[1], deg=True)


# =========================
# LCD FUNCTION
# =========================

def lcd_print(l1="", l2="", l3="", l4=""):
    def f(x): return str(x)[:16].ljust(16)

    lcd.cursor_pos = (0, 0); lcd.write_string(f(l1))
    lcd.cursor_pos = (0, 1); lcd.write_string(f(l2))
    lcd.cursor_pos = (0, 2); lcd.write_string(f(l3))
    lcd.cursor_pos = (0, 3); lcd.write_string(f(l4))


# =========================
# FAULT DIRECTION
# =========================

def is_forward(i, v):
    phase_diff = fft_phase(i) - fft_phase(v)
    phase_diff = (phase_diff + 180) % 360 - 180
    return abs(phase_diff) <= FORWARD_HALF_ANGLE


# =========================
# TRIP
# =========================

def trip():
    GPIO.output(TRIP_PIN, GPIO.HIGH)
    time.sleep(0.1)
    GPIO.output(TRIP_PIN, GPIO.LOW)


# =========================
# MAIN LOOP
# =========================

def run():
    last_update = 0
    LCD_RATE = 0.2

    lcd_print("Relay READY", "ADS1115 ACTIVE", "", "")

    while True:
        i, v = sample_cycle()
        i_rms = fft_rms(i)

        fault = i_rms > SETTING_CURRENT_RMS

        if time.time() - last_update > LCD_RATE:
            if fault:
                lcd_print(
                    "FAULT DETECTED",
                    f"I={i_rms:.2f}A",
                    f"Mode={RELAY_MODE}",
                    "Analyzing..."
                )
            else:
                lcd_print(
                    "NORMAL",
                    f"I={i_rms:.2f}A",
                    "Monitoring",
                    ""
                )
            last_update = time.time()

        if fault and is_forward(i, v):
            lcd_print("TRIP", "FORWARD FAULT", "BREAKER OPEN", "")
            trip()
            time.sleep(1)


# =========================
# ENTRY
# =========================

if __name__ == "__main__":
    try:
        run()
    except KeyboardInterrupt:
        GPIO.cleanup()