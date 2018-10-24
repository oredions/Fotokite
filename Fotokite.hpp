/* 
 * File:   Fotokite.hpp
 * Author: Jan Dufek
 *
 * Created on April 27, 2017, 2:20 PM
 */

#ifndef FOTOKITE_HPP
#define FOTOKITE_HPP

#include "opencv2/core.hpp"
#include <stdio.h>
#include <iostream>
#include <fstream>
#include <string>
#include <cstring>
#include <cmath>
#include <limits>
#include <stack>
#include <vector>
#include <deque> 

#include "config.h"
#include "SocketCommunication.hpp"
#include "SerialCommunication.hpp"
#include "ContactPoint.hpp"

using namespace std;
using namespace cv;

class Fotokite {
public:
    Fotokite(const char *, const short, const short);
    Fotokite(const char *);
    Fotokite(const Fotokite& orig);
    virtual ~Fotokite();
    
    // Ground status getters
    int getGroundMode();
    double getRuntimeS();
    double getGroundBattVoltage();
    double getRelTetherLength();

    // Attitude getters
    double getQX();
    double getQY();
    double getQZ();
    double getQW();
    
    // Gimbal getters
    double getGimbalQX();
    double getGimbalQY();
    double getGimbalQZ();
    double getGimbalQW();
    
    // Orientation getters
    double getYaw();
    double getPitch();
    double getRoll();
    
    // Gimbal orientation getters
    double getGimbalYaw();
    double getGimbalPitch();
    double getGimbalRoll();

    // Position getters
    double getElevation();
    double getRelAzimuth();
    double getBaroAlt();

    // Flight status getters
    int getFlightMode();
    double getOnTime();
    double getBackup();
    unsigned int getFlags();

    // Takeoff and landing
    void disableTetherController();
    void enableTetherController();
    void startMotors();
    void takeoff();
    void stopMotors();
    void land();
    
    // Gimbal control
    void gimbal(double, double);
    void gimbalRoll(double);
    void gimbalPitch(double);

    // Position control
    void pos(double, double, double);
    void pos2(double, double, double);
    void posV(double);
    void posH(double);
    void posL(double);

    // Yaw control
    void yaw(double);

    // Stop Fotokite
    void stop();
    
    // Print current state
    void printState();

    // Waypoint navigation
    void goToWaypoint(double waypoint[9]);
    void goToWaypoint(double, double, double, double, double, double, double, double, double);
    void executePath(string);
    void executePath();
    
    // Control
    void positionControl(double targetTetherLength, double targetElevation, double targetAzimuth, double tetherTolerance, double ElevationTolerance, double azimuthTolerance, double tetherRate, double elevationRate, double azimuthRate);
    void velocityControl(double x, double y, double z, double currentTetherLength, double currentElevation, double currentAzimuth, double currentX, double currentY, double currentZ, double &, double &, double &, double &);

    // Coordinates transformation
    void rearrangeCoordinates(double& x, double& y, double& z, double& contactPointX, double& contactPointY, double& contactPointZ);
    void scaleCoordinates(double& x, double& y, double& z, double& contactPointX, double& contactPointY, double& contactPointZ, double);

private:

    FotokiteState * state;

    Communication * communication;
    
    // Log file
    ofstream logFile;

    // Stack of contact points
    stack<ContactPoint> contactPoints;

    void updateContactPoints(double, double, double);
    void addContactPoint(double, double, double);

    struct TetherLength {
        double total;
        double segment;
    };

    struct TetherLength computeStaticTetherLength(double, double, double);

    string getCurrentTime();

    void sendCommand(string);

    // Log
    void log(double, double, double, double, double, double, double, double, double, double, double, double, double, double, double, double, double, double, double, double, double, bool, double, double, double, double, double, double, double, double, double, double);
    
    double correctInputValues(double, double);

    double getScaledTetherLength();

    // Low level control
    double tetherControl(double, double);
    double elevationControl(double, double);
    double azimuthControl(double, double);
    
    // View control
    bool viewControl(double, double, double, double, double, double, double, double, double &, double &, double &, double &, double &, double &);
    bool yawControl(double, double, double, double, double, double &, double &, double &);
    bool gimbalPitchControl(double, double, double, double, double, double, double, double &, double &, double &);

    // Corrected elevation
    double getCorrectedElevation();
//    void updateTetherEndpoint();
//    double getTetherTension();
//    double getLeanAngle();
//    
//    double tetherEndpointX;
//    double tetherEndpointY;
//    
//    // Physical properties
//    double tetherLinearDensity = 0.0061; // kg / m
//    double airframeWeight = 0.618; // kg
//    
    
    // Tether length scale
    
    // Tether unit transform. 100 ticks is 0.406 m (measured using 1 experiment while sending 100 ticks one times). Therefore, 1 tick is 0.406/100 m.
    //double tetherScale = 0.406 / 100;

    // Tether unit transform. 1000 ticks is 2.85 m (measured using 5 experiments while sending 100 ticks ten times). Therefore, 1 tick is 2.85/1000 m. Unit is meter per tick.
    const double TETHER_SCALE = 0.00285;
    
    // Tether length velocity at which the tether length does not change
//    const double TETHER_NO_MOTION_RATE = -440;
    const double TETHER_NO_MOTION_RATE = -160;
    
    // Width of dead zone centered in TETHER_NO_MOTION_RATE. Tether is not moving in this range.
//    const double DEAD_ZONE_WIDTH = 140;
    const double DEAD_ZONE_WIDTH = 80;
    
};

#endif /* FOTOKITE_HPP */

