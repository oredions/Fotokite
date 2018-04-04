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

#include "SocketCommunication.hpp"
#include "SerialCommunication.hpp"
#include "ContactPoint.hpp"

using namespace std;
using namespace cv;

class Fotokite {
public:
    Fotokite(const char *, const short);
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
    
    // Orientation getters
    double getYaw();
    double getPitch();
    double getRoll();

    // Position getters
    double getElevation();
    double getRelAzimuth();
    double getBaroAlt();

    // Flight status getters
    int getFlightMode();
    double getOnTime();
    double getBackup();
    unsigned int getFlags();

    // Gimbal control
    void gimbal(double, double);
    void gimbalRoll(double);
    void gimbalPitch(double);

    // Position control
    void pos(double, double, double);
    void posV(double);
    void posH(double);
    void posL(double);

    // Yaw control
    void yaw(double);

    // Print current state
    void printState();

    // Waypoint navigation
    void goToWaypoint(double, double, double, double, double, double, double, double, double);
    void executePath(string);
    
    // Control
    void positionControl(double targetTetherLength, double targetElevation, double targetAzimuth, double tetherTolerance, double ElevationTolerance, double azimuthTolerance, double tetherRate, double elevationRate, double azimuthRate);
    void velocityControl(double x, double y, double z, double currentTetherLength, double currentElevation, double currentAzimuth, double currentX, double currentY, double currentZ);

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
    void log(double, double, double, double, double, double, double, double, double, double, double, double, double, double, double, double, double, double, double, double, double);

    double correctInputValues(double, double);

    double getScaledTetherLength();

    // Low level control
    double tetherControl(double, double);
    double elevationControl(double, double);
    double azimuthControl(double, double);

};

#endif /* FOTOKITE_HPP */

