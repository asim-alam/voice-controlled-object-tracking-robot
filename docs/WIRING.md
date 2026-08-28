# Wiring reference

Verify every voltage level against your exact board revision before powering
the robot. Connect the grounds of all controllers and power supplies.

## Controller-to-controller signals

| Source | Destination | Purpose |
|---|---|---|
| ESP32-S3 GPIO17 | ESP32-CAM GPIO13 | Active-HIGH voice trigger |
| ESP32-S3 GPIO17 | Drive Arduino D4 | Active-HIGH scan trigger |
| ESP32-CAM GPIO2 | Drive Arduino D12 | Target centered pulse |

The signal connections also require a common ground between all three boards.

## Voice/audio ESP32-S3

| Module signal | ESP32-S3 GPIO |
|---|---:|
| INMP441 WS/LRCLK | 15 |
| INMP441 SCK/BCLK | 14 |
| INMP441 SD/DOUT | 13 |
| MAX98357 BCLK | 20 |
| MAX98357 LRC | 21 |
| MAX98357 DIN | 47 |

## Drive Arduino (documented tracking path)

| Device signal | Arduino pin |
|---|---:|
| Voice trigger | D4 |
| Camera centered pulse | D12 |
| Scan servo | A0 |
| HC-SR04 TRIG | 5 |
| HC-SR04 ECHO | 3 |
| MPU6050 SDA / SCL | A4 / A5 |
| Left motor IN1 / IN2 / ENA | 6 / 7 / 9 |
| Right motor IN3 / IN4 / ENB | 8 / 10 / 11 |

These assignments match an Arduino Uno/Nano-style 5 V board. Check the board's
I2C and PWM pin mapping if you use another model. On the photographed four-wheel
chassis, wire the two left motors to the left driver channel and the two right
motors to the right channel only if the driver and supply are rated for the
combined current.

## Optional ESP32/Blynk drive port

| Device signal | ESP32 GPIO |
|---|---:|
| Voice trigger | 4 |
| Camera centered pulse | 12 |
| Scan servo | 18 |
| HC-SR04 TRIG / ECHO | 5 / 13 |
| MPU6050 SDA / SCL | 21 / 22 |
| Left motor IN1 / IN2 / ENA | 27 / 26 / 25 |
| Right motor IN3 / IN4 / ENB | 33 / 32 / 17 |

GPIO12 is a boot-strapping pin on many ESP32 boards. If boot fails, move the
camera input to a suitable GPIO and update `INPUT_DETECTED_PIN`. The HC-SR04
ECHO output is normally 5 V, so level-shift it before GPIO13 or any other 3.3 V
ESP32 input.

## Power notes

- Do not power motors or the servo from a microcontroller GPIO or 3.3 V pin.
- Use a motor supply appropriate for the motor/driver ratings. A 7.4 V battery
  may suit the motor side if every motor and the driver support it.
- Power the servo from a separate regulated rail within its specified range
  (commonly 5-6 V), not directly from a 7.4 V battery.
- Power each controller through a supported USB, 5 V, or VIN input; never assume
  the same raw battery voltage is safe for every board.
- Add bulk capacitance near the motor driver and servo rail.
- Establish a common ground before connecting GPIO signal wires.
