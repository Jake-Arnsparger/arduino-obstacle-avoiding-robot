# arduino-obstacle-avoiding-robot

A small two-wheeled robot that drives forward, scans left and right with a servo-mounted ultrasonic sensor when it sees an obstacle, and turns toward the clearer side. An IMU provides heading so turns are measured by angle, not just by time.

Hardware:
- Arduino board model
- Ultrasonic distance sensor on a pan servo
- MPU IMU over I2C (address 0x68)
- Motor driver mode and two DC motors

How It Works:
The robot runs as a small state machine:

State: What it does
FORWARD: Drives straight and polls distance at 20 Hz

DECIDE_TURN: Stops, pans the servo right and left, picks the side with more space

TURNING: Spins until it reaches the target angle, the path is clear, or a timeout hits

REVERSE_BACKOUT: If both sides are blocked, backs up briefly and makes a U-turn

Sensor handling:


Heading:A Mahony filter fuses gyro and accelerometer data to estimate yaw. The gyro is calibrated at startup.

Bad-reading safeguards: Failed IMU reads are skipped instead of being fed to the filter. An obstacle must be seen on two consecutive readings before the robot reacts, and repeated no-echo readings from the distance sensor are treated as a problem instead of a clear path.

Turn fail-safe:Every turn has a timeout, so a drifting yaw estimate can't spin the robot forever.

Results
Worked in testing, including with the distance sensor blocked. [Add one line on what the robot did, for example "it stopped and scanned instead of driving on."

Files
- Diagnosis.ino: the full sketch
 Setup
1. Wire the sensors and motor driver to the pins defined at the top of the sketch.
2. Open the sketch in the Arduino IDE and upload it to the board.
3. Keep the robot still while it calibrates the gyro at startup.

Credit:
Jake Arnsparger
