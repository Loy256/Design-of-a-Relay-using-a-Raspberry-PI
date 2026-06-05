#!/usr/bin/env python3
"""
LCD Testing Script for Raspberry Pi
Tests I2C LCD connectivity and functionality
"""

import time
import board
import busio
from RPLCD.i2c import CharLCD

# =========================
# I2C SETUP
# =========================
print("=" * 50)
print("LCD TEST SCRIPT")
print("=" * 50)

i2c = busio.I2C(board.SCL, board.SDA)

# =========================
# SCAN I2C BUS
# =========================
print("\nScanning I2C bus for all devices...")
detected_devices = []

for address in range(0x03, 0x78):
    try:
        i2c.writeto(address, bytes([0]))
        detected_devices.append(address)
        print(f"  ✓ Device found at address: {hex(address)}")
    except (OSError, IOError):
        pass

if not detected_devices:
    print("  ✗ No I2C devices found!")
else:
    print(f"\nTotal devices found: {len(detected_devices)}")
    print(f"Addresses: {[hex(a) for a in detected_devices]}")

# =========================
# TEST LCD AT COMMON ADDRESSES
# =========================
print("\n" + "=" * 50)
print("Testing LCD at common PCF8574 addresses...")
print("=" * 50)

lcd = None
lcd_address = None
common_addresses = [0x27, 0x3F, 0x20, 0x21]

for addr in common_addresses:
    try:
        print(f"\nAttempting to initialize LCD at {hex(addr)}...")
        
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
        print(f"✓ SUCCESS! LCD initialized at {hex(addr)}")
        break
        
    except (OSError, IOError) as e:
        print(f"✗ Failed at {hex(addr)}: {e}")
        lcd = None

# =========================
# TEST LCD AT DETECTED ADDRESSES (if not common)
# =========================
if lcd is None and detected_devices:
    print("\n" + "=" * 50)
    print("LCD not found at common addresses.")
    print("Trying detected I2C addresses (excluding 0x48 ADS1115)...")
    print("=" * 50)
    
    for addr in detected_devices:
        if addr not in common_addresses and addr != 0x48:
            try:
                print(f"\nAttempting to initialize LCD at {hex(addr)}...")
                
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
                print(f"✓ SUCCESS! LCD initialized at {hex(addr)}")
                break
                
            except (OSError, IOError) as e:
                print(f"✗ Failed at {hex(addr)}: {e}")
                lcd = None

# =========================
# LCD FUNCTIONALITY TEST
# =========================
if lcd is not None:
    print("\n" + "=" * 50)
    print(f"LCD FUNCTIONALITY TEST (Address: {hex(lcd_address)})")
    print("=" * 50)
    
    # Test 1: Clear screen
    print("\nTest 1: Clearing screen...")
    lcd.clear()
    time.sleep(0.5)
    print("✓ Screen cleared")
    
    # Test 2: Write to all 4 lines
    print("\nTest 2: Writing to all 4 lines...")
    lines = [
        "Line 1: Testing!",
        "Line 2: I=1.50A",
        "Line 3: V=230V",
        "Line 4: OK!"
    ]
    
    for i, line_text in enumerate(lines):
        lcd.cursor_pos = (i, 0)
        lcd.write_string(line_text[:16])
        print(f"  ✓ Line {i+1}: {line_text}")
    
    time.sleep(2)
    
    # Test 3: Test cursor positioning
    print("\nTest 3: Testing cursor positioning...")
    lcd.clear()
    
    lcd.cursor_pos = (0, 0)
    lcd.write_string("Top-Left")
    
    lcd.cursor_pos = (0, 8)
    lcd.write_string("Top-Right")
    
    lcd.cursor_pos = (3, 0)
    lcd.write_string("Bottom")
    
    print("✓ Cursor positioning works")
    time.sleep(2)
    
    # Test 4: Scrolling test
    print("\nTest 4: Scrolling text test...")
    lcd.clear()
    
    messages = [
        "Scrolling Test 1",
        "Scrolling Test 2",
        "Scrolling Test 3",
        "Scrolling Test 4"
    ]
    
    for msg in messages:
        lcd.clear()
        lcd.cursor_pos = (0, 0)
        lcd.write_string(msg[:16])
        time.sleep(1)
    
    print("✓ Scrolling works")
    
    # Test 5: Final status screen
    print("\nTest 5: Displaying final status...")
    lcd.clear()
    
    lcd.cursor_pos = (0, 0)
    lcd.write_string("LCD TEST")
    
    lcd.cursor_pos = (1, 0)
    lcd.write_string(f"Addr: {hex(lcd_address)}")
    
    lcd.cursor_pos = (2, 0)
    lcd.write_string("PASSED!")
    
    lcd.cursor_pos = (3, 0)
    lcd.write_string("All tests OK")
    
    print("✓ Final status displayed")
    
    # Summary
    print("\n" + "=" * 50)
    print("LCD TEST COMPLETED SUCCESSFULLY!")
    print("=" * 50)
    print(f"\nLCD Details:")
    print(f"  Address: {hex(lcd_address)}")
    print(f"  Columns: 16")
    print(f"  Rows: 4")
    print(f"  Expander: PCF8574")
    print(f"\nYour relay_fixed.py should work correctly!")
    print(f"Update the address in relay_fixed.py if needed.")
    
    # Keep the LCD showing the test message
    print("\nLCD will display test message for 10 seconds...")
    time.sleep(10)
    
    lcd.clear()
    lcd.cursor_pos = (0, 0)
    lcd.write_string("Test Complete")
    lcd.cursor_pos = (1, 0)
    lcd.write_string("Ready for relay")

else:
    # LCD not found
    print("\n" + "=" * 50)
    print("LCD TEST FAILED")
    print("=" * 50)
    print("\n⚠ LCD COULD NOT BE DETECTED!")
    print("\nPossible causes:")
    print("  1. LCD is not connected to I2C bus")
    print("  2. LCD is powered off")
    print("  3. LCD is on I2C-0 instead of I2C-1")
    print("  4. LCD address is not in common range")
    print("  5. LCD PCF8574 expander is defective")
    
    print("\nDetected I2C devices on this bus:")
    if detected_devices:
        for addr in detected_devices:
            if addr == 0x48:
                print(f"  {hex(addr)} - ADS1115 (ADC)")
            else:
                print(f"  {hex(addr)} - Unknown device")
    else:
        print("  None (only ADS1115 expected)")
    
    print("\nNext steps:")
    print("  1. Check physical connections")
    print("  2. Verify LCD power supply")
    print("  3. Try 'i2cdetect -y 1' to manually scan bus")
    print("  4. Check if LCD needs to be on I2C-0 instead")

print("\n" + "=" * 50)
print("Test script finished")
print("=" * 50)
