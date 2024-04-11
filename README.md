# Smart Robot Car

### A Robot car built with Arduino uno and L293D shield.
![Project Picture](https://github.com/nawara2007/Smart-Robot-Car/assets/138627794/e3c65764-4f9d-49c7-aeb5-6d573303c793)

### Features in this project:
 1. A robot car controlled with ir remote control.
 2. Mode1 --> Obstacle avoidance by always going forward unless something is sensed near to the car
              by the ultrasonic sensor so it decide to turn in which direction by sensing the distance
              on the left and the right of the car and go in the more distant space direction.
 3. Mode2 --> Line tracking by sensing a dark line under the car by 3 line tracking modules

### What is in this repository
 1. The schematic for the project
 2. A simulation project on proteus for it
 3. The libraries used in proteus for simulation : IRX (For infrared remote), InfraredTrackerSensorTEP( fro line tracking module)
 4. The source code written by me for this project
 
### Used components:
 1. Arduino UNO
 2. L293D shield
 3. x4 Motors with gearbox
 4. Servo motor (SG90)
 5. Ulrasonic sensor (HC-SR04)
 6. x3 Line tracking sensor
 7. IR receiver module (TSOP38)
 8. x2 Li-ion battery (18500)
 9. ON-OFF switch
 10. 2x Red LEDs, 2x Yellow LEDs
 11. x4 330 resistors

### Used Libraries in arduino:
 1. [MotorDriver](https://github.com/CuriosityGym/MotorDriver)
 2. [TinyIrReceiver](https://github.com/Arduino-IRremote/Arduino-IRremote)
 3. [Servo](https://github.com/arduino-libraries/Servo)
 4. [NewPing](https://github.com/eliteio/Arduino_New_Ping)

### IR remote key bindings
![Key Bindings](https://github.com/nawara2007/Smart-Robot-Car/assets/138627794/51ce0d27-a63a-44e9-be0d-8003e73a2f4a)

### A Test video

https://github.com/nawara2007/Smart-Robot-Car/assets/138627794/f411e45a-7b1b-464e-b3d1-851a03d6b008


