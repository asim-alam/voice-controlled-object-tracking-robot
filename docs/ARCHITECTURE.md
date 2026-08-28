# System architecture

The documented prototype path distributes work across two ESP32-family boards,
an Arduino drive controller, and one host PC.

```mermaid
flowchart LR
    Mic[INMP441 microphone] --> Voice[ESP32-S3 voice and audio controller]
    Voice -- 16 kHz PCM / UDP 12345 --> Host[Python voice bridge]
    Host -- command / UDP 12346 --> Voice
    Host -- spoken reply / UDP 12348 --> Voice
    Voice --> Speaker[MAX98357 and speaker]
    Voice -- BAG pulse / GPIO17 --> Vision[ESP32-CAM vision controller]
    Voice -- same BAG pulse --> Drive[Arduino drive controller]
    Vision -- centered target / GPIO2 --> Drive
    Drive --> Servo[Scanning servo and ultrasonic sensor]
    Drive --> Motors[Motor driver and paired left/right motors]
    Drive <--> MPU[MPU6050 heading feedback]
```

## Tracking sequence

1. The ESP32-S3 streams microphone samples to the host computer.
2. The Python bridge recognizes speech and sends an uppercase command back.
3. For `BAG`, the ESP32-S3 raises GPIO17 for one second.
4. That pulse starts both the camera detector and the drive controller's servo
   scan.
5. The ESP32-CAM runs the Edge Impulse model. When a confident target is in the
   center region, it pulses GPIO2.
6. The drive controller stops the scan, aligns the chassis using the MPU6050,
   drives toward the target, and stops within the configured ultrasonic range.

## Current limitations

- The host bridge uses online Google speech recognition.
- Conversational AI and TTS are optional; command recognition works without a
  Gemini key.
- `FORWARD`, `BACKWARD`, `LEFT`, `RIGHT`, and `STOP` are recognized by the host,
  but the current voice firmware only logs those commands.
- The optional ESP32/Blynk drive controller implements the same automatic
  tracking sequence and adds manual Blynk movement controls while idle. It uses
  a different pin map from the documented Arduino tracking path.
- The prototype expects all controllers to share a common ground.
