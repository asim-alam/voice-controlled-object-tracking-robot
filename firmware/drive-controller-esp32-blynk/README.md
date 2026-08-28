# Optional ESP32/Blynk drive controller

This sketch is an alternative to the Arduino drive controller. It implements
the voice-triggered servo scan, camera-centered stop signal, MPU6050 alignment,
ultrasonic approach stop, alignment timeout, and forward-motion timeout. Blynk
manual controls are available only while the automatic state machine is idle.

Install Blynk, ESP32Servo, and a compatible MPU6050/I2Cdev library. Copy
`secrets.example.h` to `secrets.h`, open
`drive-controller-esp32-blynk.ino`, and review the GPIO12 boot-strapping and
HC-SR04 ECHO level-shifting warnings in the sketch and wiring guide.
