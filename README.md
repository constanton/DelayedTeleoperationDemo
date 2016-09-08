# Delayed Teleoperation Demo
A modified version of the "shapes" example from Chai3D using Boost C++ libraries to demonstrate the impact of constant delay in teleoperation. Tested with the IEEE-1394 (aka FireWire) version of Sensable Technologies PHANTOM Omni(R).

## How to use

| Controls        | Actions     | 
| :-------------: |:-------------:| 
| +     | Increase latency | 
| -       | Decrease latency | 

## Known Issues
* When you concanstly press the + or - keys to increase/decrease delay, the program sometimes crashes.

## Ways to improve it
* Fix known Issues (duh!)
* Implement stability control algorithms (eg. passivity-based control).
* Make the Phantom Omni run under Linux. No coding needed, but the IEEE-1394 version just won't work for me under Linux.
* Suggest other ways to improve it

## License
The code is licenced under the Revised BSD License (3-clause) according to [http://www.chai3d.org/download/license]
