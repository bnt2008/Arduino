## Arduino

This is a set of libraries and arduino programs for a "trashbot"

See the wiki for more information



The project contains 3 microprocessors:

## Spine: 
Connected by I2C to a 6DOF gyroscope
Controls motor output through an H-bridge motor controller
reports stauts to the Brain via Serial connection

## Ears
Recieves analog input from a pair of electret microphones
Also controls a simple multicolored LED display
Connects to Brain via I2C

## Brain
Controls a pair of ping sensors
Is connected to Spine via Serial and Ears via I2C
Completes planning of movement and controls motor by sending coded bytes to the Spine


## Remaining are libraries used in the sketches
