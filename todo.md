

# Badger TODO

## Servos
* DONE - voltage check
* fault detection on all servo commands
* check for servo error condition
* servo PID parameters and torque
* monitor nominal torque and use:
  * threshold for detecting collisions
  * indication of ground contact


## LIDAR
* DONE - LIDAR integration
* DONE - LIDAR based simple obstacle avoidance
* Fix getting stuck in corners
* Enable going backward if boxed in
* explore tilt up/down to improve chance of seeing low/high obstacles


## Gaits
* DONE - fix bug with transition to passive
* check stride length correctly applied... is toe down forward of rest?
* Fix one and two leg gaits
* Fix transition stand to walk



## Web
* DONE - Why does a web request halt the gait updates?
* Better web interface
  * Controls for manual walking



## OpenMV Camera
* Design neck
* How to wire up the camera comms?


## IMU
* Integrate MPU6050
