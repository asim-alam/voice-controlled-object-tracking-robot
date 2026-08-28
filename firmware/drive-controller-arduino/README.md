# Arduino drive controller

This is the documented tracking controller. It waits for the one-second voice
trigger on D4, scans the servo until the camera pulses D12, records the servo
angle, aligns the chassis using the MPU6050, approaches the target, and stops at
15 cm according to the ultrasonic sensor. If alignment cannot complete within
10 seconds or no stop distance is reached within 15 seconds of forward motion,
the motors are stopped and the controller returns to idle.

The pin map and required libraries are described in
[`../../docs/WIRING.md`](../../docs/WIRING.md).
