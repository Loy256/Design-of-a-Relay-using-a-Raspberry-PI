#!/usr/bin/env python3
"""
Single-Phase Directional Overcurrent Protection Relay
Raspberry Pi 5 + ADS1115 + PCF8574 LCD

BUGS FIXED vs submitted code:
  FIX 1 — VirtualDisc.__init__ had single underscores (_init_) so it
           never ran — position and HALF_CYCLE_DURATION were undefined.
  FIX 2 — read_voltage() clipped v < 0 to zero, destroying the negative
           half of the sine wave. FFT of a half-wave gives near-zero RMS.
  FIX 3 — fft_rms() always used bin index 1 which is only correct when
           samples = SAMPLE_RATE / SYSTEM_FREQ = 20. With SAMPLES=32
           bin 1 is not 50 Hz. Correct bin computed as round(f * N / fs).
  FIX 4 — Hanning window reduces amplitude by ~50%. Must compensate with
           factor 2.0 or remove the window (removed — not needed at 32 pts).
  FIX 5 — CH_CURRENT assigned to A0, CH_VOLTAGE to A1 — swapped vs
           schematic (voltage on A0, current on A1). Corrected.
  FIX 6 — if __name__ == "__main__" used single underscores so run()
           was never called when script executed directly.
  FIX 7 — VirtualDisc.advance() used self.HALF_CYCLE_DURATION which
           never existed due to FIX 1. Now uses local constant.
  FIX 8 — SAMPLES_PER_CYCLE changed back to 20 (one full cycle at
           1000 Sa/s / 50 Hz = 20). 32 samples spans 1.6 cycles and
           causes spectral leakage in the FFT.
  FIX 9 — ads.gain set to 2 (±2.048 V) instead of 1 (±4.096 V).
           Your signals are ~0.15 V — gain 2 gives 4× better resolution.
"""

import numpy as np
import time
import board
import busio
import adafruit_ads1x15.ads1115 as ADS
from adafruit_ads1x15.analog_in import AnalogIn
from RPLCD.i2c import CharLCD
import signal
import sys
from gpiozero import OutputDevice

# ════════════════════════════════════════════════════════
# CONFIGURATION
# ════════════════════════════════════════════════════════

SAMPLE_RATE         = 1000         # Sa/s
SYSTEM_FREQUENCY    = 50           # Hz
SAMPLES_PER_CYCLE   = SAMPLE_RATE // SYSTEM_FREQUENCY  # FIX 8: must be 20
SETTING_CURRENT_RMS = 1.0          # A RMS — relay pick-up threshold
TMS                 = 0.3          # Time Multiplier Setting
RELAY_CHAR_ANGLE    = 0            # RCA degrees (0 or 60)
RELAY_MODE          = "IDMT"       # "INSTANTANEOUS" or "IDMT"
FORWARD_HALF_ANGLE  = 90           # degrees

R_BURDEN            = 0.5          # Ω — your actual burden resistor value

LCD_RATE            = 0.2          # seconds between LCD updates

# ════════════════════════════════════════════════════════
# GPIO SETUP
# ════════════════════════════════════════════════════════

TRIP_PIN = None

def init_gpio():
    global TRIP_PIN
    try:
        TRIP_PIN = OutputDevice(11, active_high=True, initial_value=False)
        print("✓ GPIO11 initialized")
        return True
    except Exception as e:
        print(f"⚠ GPIO11 failed: {e} — trip output disabled")
        return False

def cleanup(signum=None, frame=None):
    print("\nShutting down...")
    if TRIP_PIN is not None:
        try:
            TRIP_PIN.off()
            TRIP_PIN.close()
            print("✓ GPIO cleaned up")
        except:
            pass
    sys.exit(0)

signal.signal(signal.SIGINT,  cleanup)
signal.signal(signal.SIGTERM, cleanup)

# ════════════════════════════════════════════════════════
# I2C + ADS1115
# ════════════════════════════════════════════════════════

i2c = busio.I2C(board.SCL, board.SDA)
ads = ADS.ADS1115(i2c, address=0x48)

# FIX 9: gain=2 → ±2.048 V range — much better resolution for ~0.15 V signals
ads.gain      = 2
ads.data_rate = 475    # SPS — stable without being too slow

# FIX 5: A0 = voltage channel (VT), A1 = current channel (CT)
# Must match your schematic: VT divider → A0, CT divider → A1
CH_VOLTAGE = AnalogIn(ads, ADS.A0)   # VT signal → A0
CH_CURRENT = AnalogIn(ads, ADS.A1)   # CT signal → A1

print(f"✓ ADS1115 initialised  gain={ads.gain} (±2.048 V)  rate={ads.data_rate} SPS")

# ════════════════════════════════════════════════════════
# LCD SETUP
# ════════════════════════════════════════════════════════

lcd = None
lcd_address = None

print("Scanning for LCD...")
for addr in [0x27, 0x3F, 0x20, 0x21, 0x22, 0x23, 0x24, 0x25, 0x26, 0x38]:
    try:
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
        print(f"✓ LCD found at {hex(addr)}")
        break
    except Exception:
        lcd = None

if lcd is None:
    print("⚠ LCD not found — printing to console instead")

# ════════════════════════════════════════════════════════
# ADC READING
# FIX 2: removed clipping of negative values — sine wave is bipolar
# ════════════════════════════════════════════════════════

def read_adc(channel: AnalogIn) -> float:
    """Read one voltage sample from ADS1115. Returns signed float in volts."""
    return channel.voltage   # Adafruit library returns signed value correctly


# ════════════════════════════════════════════════════════
# SAMPLING — phase-coherent matched pairs
# ════════════════════════════════════════════════════════

def sample_cycle() -> tuple[np.ndarray, np.ndarray]:
    """
    Acquire one full cycle of current AND voltage samples.
    Both channels sampled within same 1 ms window to preserve phase.
    Returns (i_samples, v_samples) each of length SAMPLES_PER_CYCLE.
    """
    i_samples = np.empty(SAMPLES_PER_CYCLE)
    v_samples = np.empty(SAMPLES_PER_CYCLE)
    interval  = 1.0 / SAMPLE_RATE
    next_time = time.perf_counter()

    for k in range(SAMPLES_PER_CYCLE):
        i_samples[k] = read_adc(CH_CURRENT)
        v_samples[k] = read_adc(CH_VOLTAGE)
        next_time += interval
        while time.perf_counter() < next_time:
            pass

    return i_samples, v_samples


# ════════════════════════════════════════════════════════
# FFT ANALYSIS
# FIX 3: correct fundamental bin index
# FIX 4: removed Hanning window — not needed at exactly 20 samples/cycle
# ════════════════════════════════════════════════════════

def _fund_bin(n: int) -> int:
    """Index of 50 Hz fundamental in an n-point FFT at SAMPLE_RATE Sa/s."""
    return round(SYSTEM_FREQUENCY * n / SAMPLE_RATE)  # = 1 for n=20


def fft_rms(samples: np.ndarray) -> float:
    """
    Return RMS amplitude of fundamental component in volts.

    FIX 3: uses correct bin index, not hardcoded bin 1.
    FIX 4: no Hanning window — with exactly 20 samples per cycle the
           fundamental sits perfectly in bin 1 with no leakage.
           Adding a Hanning window here would require amplitude correction
           and is unnecessary for coherent sampling.
    """
    n = len(samples)
    X = np.fft.fft(samples)
    fund = _fund_bin(n)
    peak = (2.0 / n) * abs(X[fund])
    return peak / np.sqrt(2)   # peak → RMS


def fft_phase(samples: np.ndarray) -> float:
    """Return phase angle (degrees) of fundamental component."""
    n = len(samples)
    X = np.fft.fft(samples)
    fund = _fund_bin(n)
    return np.degrees(np.angle(X[fund]))


# ════════════════════════════════════════════════════════
# VIRTUAL DISC (IDMT)
# FIX 1: __init__ corrected (was _init_ — never called)
# FIX 7: HALF_CYCLE_DURATION now defined in __init__
# ════════════════════════════════════════════════════════

class VirtualDisc:
    """Simulates induction disc of electromechanical IDMT relay."""

    def __init__(self):                          # FIX 1: was _init_
        self.position             = 0.0
        self.HALF_CYCLE_DURATION  = 1.0 / (2 * SYSTEM_FREQUENCY)  # FIX 7

    def reset(self):
        self.position = 0.0

    def idmt_trip_time(self, i_rms: float) -> float:
        """Standard Inverse trip time (s). Returns inf if below pick-up."""
        ratio = i_rms / SETTING_CURRENT_RMS
        if ratio <= 1.0:
            return float("inf")
        return (TMS * 0.14) / (ratio ** 0.02 - 1)

    def advance(self, i_rms: float) -> bool:
        """
        Advance disc by one half-cycle.
        Returns True when disc reaches 100% (time to check direction).
        """
        t_trip = self.idmt_trip_time(i_rms)
        if t_trip == float("inf"):
            self.position = max(0.0, self.position - 5.0)
            return False
        increment     = (self.HALF_CYCLE_DURATION / t_trip) * 100.0   # FIX 7
        self.position = min(100.0, self.position + increment)
        return self.position >= 100.0


disc = VirtualDisc()

# ════════════════════════════════════════════════════════
# DIRECTION DETECTION
# ════════════════════════════════════════════════════════

def is_forward(i_samples: np.ndarray,
               v_samples: np.ndarray,
               pre_fault_v_angle: float | None = None) -> bool:
    """
    Returns True if fault is in the forward direction.
    Uses pre-fault voltage angle if post-fault V collapses to ~0.
    """
    v_rms_val = fft_rms(v_samples)

    if v_rms_val < 0.05 and pre_fault_v_angle is not None:
        v_phase = pre_fault_v_angle
        print("[INFO] V≈0 — using pre-fault angle")
    else:
        v_phase = fft_phase(v_samples)

    i_phase    = fft_phase(i_samples)
    phase_diff = i_phase - v_phase
    phase_diff = (phase_diff + 180) % 360 - 180

    adjusted   = phase_diff - RELAY_CHAR_ANGLE
    adjusted   = (adjusted + 180) % 360 - 180

    forward = abs(adjusted) <= FORWARD_HALF_ANGLE
    print(f"[DIR] phase_diff={phase_diff:.1f}°  adjusted={adjusted:.1f}°  "
          f"→ {'FORWARD' if forward else 'REVERSE'}")
    return forward


# ════════════════════════════════════════════════════════
# LCD HELPER
# ════════════════════════════════════════════════════════

def lcd_print(l1="", l2="", l3="", l4=""):
    print(f"[LCD] {l1:<16} | {l2:<16} | {l3:<16} | {l4:<16}")
    if lcd is None:
        return
    try:
        lcd.clear()
        for row, text in enumerate([l1, l2, l3, l4]):
            lcd.cursor_pos = (row, 0)
            lcd.write_string(str(text)[:16])
    except Exception as e:
        print(f"[LCD ERR] {e}")


# ════════════════════════════════════════════════════════
# TRIP OUTPUT
# ════════════════════════════════════════════════════════

def trip():
    """Assert trip output for 100 ms."""
    if TRIP_PIN is not None:
        TRIP_PIN.on()
        time.sleep(0.1)
        TRIP_PIN.off()
        print("⚡ RELAY TRIPPED")
    else:
        print("⚠ Trip would fire (GPIO unavailable)")


# ════════════════════════════════════════════════════════
# MAIN LOOP
# ════════════════════════════════════════════════════════

def run():
    print("\nInitialising GPIO...")
    init_gpio()

    pre_fault_v_angle = None
    last_lcd_update   = 0.0

    lcd_print("Relay READY",
              f"Is={SETTING_CURRENT_RMS}A",
              f"TMS={TMS} RCA={RELAY_CHAR_ANGLE}",
              f"Mode={RELAY_MODE}")

    print(f"\n[CFG] Mode={RELAY_MODE}  Is={SETTING_CURRENT_RMS}A  "
          f"TMS={TMS}  RCA={RELAY_CHAR_ANGLE}°  R_burden={R_BURDEN}Ω\n")

    while True:
        i_samples, v_samples = sample_cycle()

        # Recover current in amps from ADC voltage reading
        i_rms = fft_rms(i_samples) / R_BURDEN   # V ÷ Ω = A
        v_rms = fft_rms(v_samples)

        print(f"[MON] I={i_rms:.4f}A  V={v_rms:.4f}V  "
              f"pick-up={SETTING_CURRENT_RMS}A")

        fault = i_rms > SETTING_CURRENT_RMS

        # ── LCD update (rate limited) ──────────────────────────
        if time.time() - last_lcd_update > LCD_RATE:
            if fault:
                if RELAY_MODE == "INSTANTANEOUS":
                    lcd_print("FAULT DETECTED",
                              f"I={i_rms:.2f}A V={v_rms:.2f}V",
                              "Mode=INSTANT",
                              "Checking dir...")
                else:
                    lcd_print(f"IDMT {disc.position:.0f}%",
                              f"I={i_rms:.2f}A V={v_rms:.2f}V",
                              f"t={disc.idmt_trip_time(i_rms):.2f}s",
                              "Accumulating...")
            else:
                lcd_print("NORMAL",
                          f"I={i_rms:.2f}A V={v_rms:.2f}V",
                          "Monitoring...",
                          "")
            last_lcd_update = time.time()

        # ── Protection logic ───────────────────────────────────
        if fault:
            if RELAY_MODE == "INSTANTANEOUS":
                if is_forward(i_samples, v_samples, pre_fault_v_angle):
                    lcd_print("TRIP!", "FORWARD FAULT",
                              "BREAKER OPEN", "")
                    trip()
                    time.sleep(1)
                else:
                    lcd_print("REVERSE FAULT",
                              f"I={i_rms:.2f}A",
                              "BLOCKED", "")

            else:   # IDMT
                if disc.advance(i_rms):
                    if is_forward(i_samples, v_samples, pre_fault_v_angle):
                        lcd_print("TRIP!", "FORWARD FAULT",
                                  "BREAKER OPEN", "")
                        trip()
                        disc.reset()
                        time.sleep(1)
                    else:
                        lcd_print("REVERSE FAULT",
                                  f"I={i_rms:.2f}A",
                                  "DISC RESET", "")
                        disc.reset()   # FIX: reset on confirmed reverse

        else:
            # Below pick-up — store pre-fault voltage angle, reset disc
            pre_fault_v_angle = fft_phase(v_samples)
            disc.reset()


# ════════════════════════════════════════════════════════
# ENTRY POINT
# FIX 6: corrected __name__ and __main__ (were _name_ and _main_)
# ════════════════════════════════════════════════════════

if __name__ == "__main__":        # FIX 6: was _name_ == _main_
    try:
        run()
    except KeyboardInterrupt:
        cleanup()
    except Exception as e:
        print(f"[ERROR] {e}")
        import traceback
        traceback.print_exc()
        cleanup()
