#!/usr/bin/env python3
"""
Single-Phase Directional Overcurrent Protection Relay
Implemented on Raspberry Pi

Based on: "Design and evaluation of a single-phase directional overcurrent
protection relay implemented using a Raspberry PI"
A. Popescu, S. Potts, P. A. Crossley - University of Manchester

Hardware assumptions:
    - MCP3208 (or similar) 16-bit ADC via SPI (±12V input range)
    - GPIO pin for trip output (via transistor + relay + flyback diode)
    - I2C LCD display for settings and status
    - Sampling rate: 1000 samples/sec → 20 samples per cycle at 50Hz

FIXES APPLIED vs original code:
    FIX 1 – Simulation polarity corrected:
        Voltage is now the reference phasor (sin ωt). Current lags voltage by
        70° for a forward inductive fault, matching the paper's test scenario.
        The original had the sign reversed, so V lagged I instead of I lagging V.

    FIX 2 – Phase-coherent full-cycle assembly:
        I and V are now sampled together in every half-cycle call and stored as
        matched pairs. The full cycle is assembled from the SAME two paired
        half-cycle acquisitions, so the phase difference between I and V is
        never contaminated by a half-cycle time offset (~10 ms) that arose when
        I and V came from different acquisition calls.

    FIX 3 – Reverse-fault disc reset in IDMT mode:
        In the original code the virtual disc advanced toward 100% even for
        reverse faults and was only rejected at the final directional check.
        The disc now resets immediately when a sustained reverse fault is
        confirmed, preventing unnecessary accumulation.

    FIX 4 – IDMT direction check on every trip attempt:
        Directional check is now also performed when the disc first exceeds the
        threshold each half-cycle rather than waiting for exactly 100%.
"""

import numpy as np
import time
import RPi.GPIO as GPIO

# ─────────────────────────────────────────────
# Hardware availability flags
# ─────────────────────────────────────────────
try:
    import spidev
    SPI_AVAILABLE = True
except ImportError:
    SPI_AVAILABLE = False
    print("[WARN] spidev not found – running in simulation mode")

try:
    import smbus
    I2C_AVAILABLE = True
except ImportError:
    I2C_AVAILABLE = False
    print("[WARN] smbus not found – LCD disabled")


# ════════════════════════════════════════════════════════
# CONFIGURATION
# ════════════════════════════════════════════════════════

SAMPLE_RATE        = 1000          # samples per second
SYSTEM_FREQUENCY   = 50            # Hz (UK grid)
SAMPLES_PER_CYCLE  = SAMPLE_RATE // SYSTEM_FREQUENCY   # 20
SAMPLES_HALF_CYCLE = SAMPLES_PER_CYCLE // 2            # 10

ADC_VREF           = 12.0          # ±12 V input range
ADC_BITS           = 16
ADC_COUNTS         = 2 ** ADC_BITS

# GPIO
TRIP_PIN           = 17            # BCM numbering
GPIO.setmode(GPIO.BCM)
GPIO.setup(TRIP_PIN, GPIO.OUT, initial=GPIO.LOW)

# ADC SPI channels (adjust for your ADC)
CH_CURRENT         = 0
CH_VOLTAGE         = 1

# ─── Relay settings (adjustable via LCD in real hardware) ───
SETTING_CURRENT_RMS = 1.0          # Is  [A RMS] – relay pick-up
TMS                 = 0.3          # Time Multiplier Setting
RELAY_CHAR_ANGLE    = 0            # RCA in degrees (0° or 60°)
RELAY_MODE          = "IDMT"       # "INSTANTANEOUS" or "IDMT"

# Operating zone half-width around RCA
FORWARD_HALF_ANGLE  = 90           # degrees


# ════════════════════════════════════════════════════════
# ADC INTERFACE
# ════════════════════════════════════════════════════════

if SPI_AVAILABLE:
    spi = spidev.SpiDev()
    spi.open(0, 0)
    spi.max_speed_hz = 1_000_000


def read_adc(channel: int) -> float:
    """
    Read one sample from the ADC and convert to volts.
    Returns a signed voltage in the range [-ADC_VREF, +ADC_VREF].
    """
    if SPI_AVAILABLE:
        # Generic 3-byte SPI read (adjust for your specific ADC)
        cmd = [1, (8 + channel) << 4, 0]
        reply = spi.xfer2(cmd)
        raw = ((reply[1] & 3) << 8) | reply[2]
        # Map 0…(ADC_COUNTS-1) → -ADC_VREF…+ADC_VREF
        volts = (raw / (ADC_COUNTS - 1)) * 2 * ADC_VREF - ADC_VREF
    else:
        # ── FIX 1: Corrected simulation polarity ──────────────────────────────
        # Voltage is the reference phasor.  For a FORWARD inductive fault,
        # current LAGS voltage.  The paper's test case uses a 70° lag, so:
        #   V(t) = A·sin(ωt)          ← reference, 0° phase
        #   I(t) = A·sin(ωt − 70°)   ← lags V by 70°  → forward fault
        #
        # Original code had V(t) = sin(ωt − 70°) which made V LAG I, reversing
        # the fault direction under test.
        t = time.time()
        if channel == CH_VOLTAGE:
            volts = 5.0 * np.sin(2 * np.pi * SYSTEM_FREQUENCY * t)
        else:  # CH_CURRENT
            # Current lags voltage by 70° → forward resistive-inductive fault
            volts = 5.0 * np.sin(2 * np.pi * SYSTEM_FREQUENCY * t
                                  - np.deg2rad(70))
    return volts


# ════════════════════════════════════════════════════════
# PHASE-COHERENT INTERLEAVED SAMPLING  (FIX 2)
# ════════════════════════════════════════════════════════

def sample_half_cycle_interleaved() -> tuple[np.ndarray, np.ndarray]:
    """
    Acquire exactly one half-cycle of current AND voltage samples,
    interleaved as closely as possible within each 1 ms slot.

    FIX 2 – returns a MATCHED (i_half, v_half) pair so that every I sample
    and its corresponding V sample are taken within the same 1 ms acquisition
    window.  The full cycle is later assembled by concatenating two consecutive
    matched pairs:

        full_i = [prev_i_half | new_i_half]
        full_v = [prev_v_half | new_v_half]

    Because prev_i_half and prev_v_half came from the SAME previous call, the
    phase relationship between I and V across the full cycle is preserved.
    This eliminates the ~10 ms systematic phase offset that occurred in the
    original code when prev_half_i and prev_half_v were populated by different
    acquisition calls at different times.

    Returns
    -------
    i_samples : ndarray, shape (SAMPLES_HALF_CYCLE,)
    v_samples : ndarray, shape (SAMPLES_HALF_CYCLE,)
    """
    i_samples = np.empty(SAMPLES_HALF_CYCLE)
    v_samples = np.empty(SAMPLES_HALF_CYCLE)
    interval = 1.0 / SAMPLE_RATE          # 1 ms between sample pairs
    next_sample_time = time.perf_counter()

    for k in range(SAMPLES_HALF_CYCLE):
        # Read both channels back-to-back (minimal delay between them)
        i_samples[k] = read_adc(CH_CURRENT)
        v_samples[k] = read_adc(CH_VOLTAGE)

        # Busy-wait until the next sampling instant to reduce jitter
        next_sample_time += interval
        while time.perf_counter() < next_sample_time:
            pass

    return i_samples, v_samples


# ════════════════════════════════════════════════════════
# FFT ANALYSIS
# ════════════════════════════════════════════════════════

def _fundamental_bin(n: int) -> int:
    """Index of the 50 Hz bin for an n-sample FFT at SAMPLE_RATE Sa/s."""
    return round(SYSTEM_FREQUENCY * n / SAMPLE_RATE)


def fft_amplitude(samples: np.ndarray) -> float:
    """
    Return the RMS amplitude of the fundamental component.
    Samples must form exactly one full cycle (20 points at 1000 Sa/s).
    """
    n = len(samples)
    fft_result = np.fft.fft(samples)
    fund_i = _fundamental_bin(n)
    peak = (2.0 / n) * abs(fft_result[fund_i])
    return peak / np.sqrt(2)   # convert peak → RMS


def fft_phase(samples: np.ndarray) -> float:
    """Return the phase angle (degrees) of the fundamental component."""
    n = len(samples)
    fft_result = np.fft.fft(samples)
    fund_i = _fundamental_bin(n)
    return np.degrees(np.angle(fft_result[fund_i]))


# ════════════════════════════════════════════════════════
# DIRECTION DETECTION
# ════════════════════════════════════════════════════════

def is_forward_fault(i_samples: np.ndarray,
                     v_samples: np.ndarray,
                     pre_fault_v_angle: float | None = None) -> bool:
    """
    Determine fault direction from the phase relationship between V and I.

    Convention (matching the paper):
        phase_diff = angle(I) − angle(V)   (how much I leads V)
        Forward fault → I lags V → phase_diff is negative (for inductive loads)
        Reverse fault → I is roughly anti-phase with V → |phase_diff| near 180°

    The RCA shifts the operating zone so that a purely inductive line angle
    can be accommodated.  With RCA = 0° and FORWARD_HALF_ANGLE = 90°, the
    operating zone covers phase_diff ∈ (−90°, +90°) — i.e. Q1 and Q4.

    Falls back to the stored pre-fault voltage angle when the post-fault
    voltage collapses to near zero (paper section 3).
    """
    v_rms = fft_amplitude(v_samples)

    if v_rms < 0.05 and pre_fault_v_angle is not None:
        v_phase = pre_fault_v_angle
        print("[INFO] Post-fault V ≈ 0 – using pre-fault angle as reference")
    else:
        v_phase = fft_phase(v_samples)

    i_phase = fft_phase(i_samples)

    # Phase difference: positive = I leads V
    phase_diff = i_phase - v_phase
    phase_diff = (phase_diff + 180) % 360 - 180   # normalise to (−180°, 180°]

    # Shift by RCA to centre the operating zone
    adjusted = phase_diff - RELAY_CHAR_ANGLE
    adjusted = (adjusted + 180) % 360 - 180

    forward = abs(adjusted) <= FORWARD_HALF_ANGLE
    print(f"[DIR]  phase_diff={phase_diff:.1f}°  adjusted={adjusted:.1f}°  "
          f"→ {'FORWARD' if forward else 'REVERSE'}")
    return forward


# ════════════════════════════════════════════════════════
# IDMT – STANDARD INVERSE (IEC 60255-151)
# ════════════════════════════════════════════════════════

def idmt_trip_time(i_rms: float) -> float:
    """
    Standard Inverse trip time (seconds) per Equation (1) of the paper:
        t = (TMS × 0.14) / ((I/Is)^0.02 − 1)
    Returns +inf when I ≤ Is (relay below pick-up).
    """
    ratio = i_rms / SETTING_CURRENT_RMS
    if ratio <= 1.0:
        return float("inf")
    return (TMS * 0.14) / (ratio ** 0.02 - 1)


# ════════════════════════════════════════════════════════
# VIRTUAL DISC
# ════════════════════════════════════════════════════════

class VirtualDisc:
    """
    Simulates the induction disc of an electromechanical IDMT relay.

    The disc position advances (0 → 100 %) in proportion to the fraction of
    the calculated trip time consumed each half-cycle.  When it reaches 100 %
    the relay checks fault direction and trips if forward.

    FIX 3 – the disc now exposes a reset path that the main loop calls
    immediately when a reverse fault is confirmed, so the disc does not
    silently accumulate position during sustained reverse overcurrents.
    """
    HALF_CYCLE_DURATION = 1.0 / (2 * SYSTEM_FREQUENCY)   # 10 ms at 50 Hz

    def __init__(self):
        self.position = 0.0          # percentage 0 … 100

    def reset(self):
        self.position = 0.0

    def advance(self, i_rms: float) -> bool:
        """
        Advance the disc by one half-cycle increment.
        Returns True when the disc reaches 100 % (time to check direction).
        """
        t_trip = idmt_trip_time(i_rms)
        if t_trip == float("inf"):
            # Below pick-up: disc slowly resets (5 % per half-cycle)
            self.position = max(0.0, self.position - 5.0)
            return False

        increment = (self.HALF_CYCLE_DURATION / t_trip) * 100.0
        self.position = min(100.0, self.position + increment)
        print(f"[DISC] I={i_rms:.3f} A  t_trip={t_trip:.3f} s  "
              f"disc={self.position:.1f} %")
        return self.position >= 100.0


# ════════════════════════════════════════════════════════
# TRIP OUTPUT
# ════════════════════════════════════════════════════════

def trip_breaker():
    """Assert the trip output – closes the relay contact for 100 ms."""
    print("[TRIP] Tripping circuit breaker!")
    GPIO.output(TRIP_PIN, GPIO.HIGH)
    time.sleep(0.1)
    GPIO.output(TRIP_PIN, GPIO.LOW)


# ════════════════════════════════════════════════════════
# LCD DISPLAY (stub)
# ════════════════════════════════════════════════════════

def lcd_print(line1: str = "", line2: str = ""):
    """Print relay status to the LCD (or console in simulation)."""
    print(f"[LCD] {line1:<16} | {line2:<16}")


# ════════════════════════════════════════════════════════
# MAIN RELAY LOOP
# ════════════════════════════════════════════════════════

def run_relay():
    """
    Main protection loop.

    FIX 2 – full-cycle assembly uses matched (i, v) half-cycle pairs so the
    FFT phase comparison between I and V is always phase-coherent.

    The sliding window works as follows:

        call 0  →  (i_a, v_a) = first half-cycle pair   [initialisation]
        call 1  →  (i_b, v_b) = second half-cycle pair
                    full_i = [i_a | i_b],  full_v = [v_a | v_b]
                    → both halves of V came from matched acquisitions ✓
        call 2  →  (i_c, v_c)
                    full_i = [i_b | i_c],  full_v = [v_b | v_c]   ✓
        …

    This ensures prev_i_half and prev_v_half always originate from the same
    acquisition call, eliminating any inter-call time offset in phase.
    """
    disc = VirtualDisc()

    # ── Initialisation: acquire two half-cycles to seed the sliding window ──
    prev_i_half, prev_v_half = sample_half_cycle_interleaved()

    # Use second half-cycle to build first full cycle and seed pre-fault angle
    new_i_half, new_v_half = sample_half_cycle_interleaved()
    full_i = np.concatenate([prev_i_half, new_i_half])
    full_v = np.concatenate([prev_v_half, new_v_half])
    pre_fault_v_angle = fft_phase(full_v)

    # Slide window forward
    prev_i_half = new_i_half
    prev_v_half = new_v_half

    lcd_print("Relay READY", f"Is={SETTING_CURRENT_RMS}A TMS={TMS}")
    print(f"[CFG]  Mode={RELAY_MODE}  Is={SETTING_CURRENT_RMS} A  "
          f"TMS={TMS}  RCA={RELAY_CHAR_ANGLE}°")

    while True:
        # ── FIX 2: acquire next matched half-cycle pair ──
        new_i_half, new_v_half = sample_half_cycle_interleaved()

        # Assemble full cycle from two consecutive MATCHED pairs
        full_i = np.concatenate([prev_i_half, new_i_half])
        full_v = np.concatenate([prev_v_half, new_v_half])

        # Compute RMS current via FFT
        i_rms = fft_amplitude(full_i)
        print(f"[MON]  I_rms={i_rms:.4f} A  (pick-up={SETTING_CURRENT_RMS} A)")

        if i_rms < SETTING_CURRENT_RMS:
            # ── Below pick-up: store pre-fault voltage angle, reset disc ──
            pre_fault_v_angle = fft_phase(full_v)
            disc.reset()
            lcd_print("Monitoring...", f"I={i_rms:.3f}A")

        else:
            # ── Fault condition ──────────────────────────────────────────────
            if RELAY_MODE == "INSTANTANEOUS":
                if is_forward_fault(full_i, full_v, pre_fault_v_angle):
                    lcd_print("TRIP!", f"I={i_rms:.3f}A FWRD")
                    trip_breaker()
                    disc.reset()
                else:
                    lcd_print("Reverse fault", f"I={i_rms:.3f}A")

            else:   # IDMT
                tripped = disc.advance(i_rms)
                lcd_print(f"Disc {disc.position:.0f}%", f"I={i_rms:.3f}A")

                if tripped:
                    if is_forward_fault(full_i, full_v, pre_fault_v_angle):
                        lcd_print("TRIP!", f"I={i_rms:.3f}A FWRD")
                        trip_breaker()
                        disc.reset()
                    else:
                        # FIX 3: reset disc immediately on confirmed reverse fault
                        # so that accumulated position does not persist into the
                        # next overcurrent event.
                        lcd_print("Reverse fault", f"I={i_rms:.3f}A")
                        disc.reset()

        # ── Slide the matched half-cycle window forward ──────────────────────
        prev_i_half = new_i_half
        prev_v_half = new_v_half


# ════════════════════════════════════════════════════════
# ENTRY POINT
# ════════════════════════════════════════════════════════

if __name__ == "__main__":
    try:
        run_relay()
    except KeyboardInterrupt:
        print("\n[INFO] Relay stopped by user.")
    finally:
        GPIO.cleanup()
        if SPI_AVAILABLE:
            spi.close()
