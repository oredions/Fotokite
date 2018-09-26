# Fotokite
This project contains C++ Fotokite Pro library. [Fotokite Pro](https://fotokite.com/fotokite-pro/) is a tethered unmanned aerial vehicle by Perspective Robotics. Due to lack of GPS, Fotokite has unique control based on tether length, tether azimuth, and tether elevation angle. This library can be used to get telemetry data from Fotokite and to send commands to Fotokite. This enables development of autonomous capabilities.

In addition to low-level commands, this library also supports waypoint navigation. Waypoints can be specified using input waypoint file. The waypoint navigation supports multiple tether contact points with obstacles. Therefore Fotokite can navigate even if the tether touches obstacles.

Two communication methods are implemented to communicate with Fotokite. The first is serial communication over USB serial port on Fotokite's ground control station. The second is socket communication using custom-built server over wireless Ethernet. The later option enables to communicate with Fotokite's ground station wirelessly, which might be advantageous if the Fotokite's ground station is attached to a moving unmanned ground vehicle.

## Publications

Details about the project can be found in the following IEEE publications:

[Visual pose stabilization of tethered small unmanned aerial system to assist drowning victim recovery](http://ieeexplore.ieee.org/document/8088149/)

[Visual servoing for teleoperation using a tethered UAV](http://ieeexplore.ieee.org/document/8088155/)

Motion planning for a UAV with a straight or kinked tether (accepted to IROS 2018)

Indoor UAV Localization Using a Tether (accepted to SSRR 2018)

## Run

1. Install CMake:

    https://cmake.org

2. In Terminal, change directory into the root directory of the project and run the following command to generate makefile:

    cmake .

3. Compile the project:

    make

4. Run:

    ./Fotokite

## Open in NetBeans

1. File

2. Open Project

3. Go to the source directory

4. Open Project
