/* 
 * File:   Fotokite.cpp
 * Author: Jan Dufek
 * 
 * Created on April 27, 2017, 2:20 PM
 */

#include "Fotokite.hpp"
#include <iostream>
#include <fstream>
#include <string>

#define PI 3.14159265

// Switch between position control and velocity control. Comment if you want position control. Uncomment if you want velocity control.
//#define VELOCITY_CONTROL

/**
 * Initialize wireless communication with the Fotokite server.
 * 
 * @param ip_address
 * @param port
 */
Fotokite::Fotokite(const char * ip_address, const short port) {

    // Initialize Fotokite state
    state = new FotokiteState();

    // Initialize communication with Fotokite OCU Server
    communication = new SocketCommunication(state, ip_address, port);

    // Initialize log
    logFile.open("log/" + getCurrentTime() + ".txt");

}

/**
 * Initialize wired serial communication with the Fotokite ground control station.
 * 
 * @param serialPort
 */
Fotokite::Fotokite(const char * serialPort) {

    // Initialize Fotokite state
    state = new FotokiteState();

    // Initialize communication with Fotokite ground station over serial port
    communication = new SerialCommunication(state, serialPort);

    // Initialize log
    logFile.open("log/" + getCurrentTime() + ".txt");

}

Fotokite::Fotokite(const Fotokite& orig) {
}

/**
 * Stop Fotokite motion, stop communication, and close log.
 */
Fotokite::~Fotokite() {

    // Stop motion
    this->gimbal(0, 0);
    this->pos(0, 0, 0);
    this->yaw(0);

    // Delete communication
    delete communication;

    // Close log
    logFile.close();

}

/**
 * Returns current time stamp in string %Y%m%d%H%M%S format.
 * 
 * @return 
 */
string Fotokite::getCurrentTime() {

    time_t rawTime;
    time(&rawTime);
    struct tm * localTime;
    localTime = localtime(&rawTime);
    char currentTime[40];
    strftime(currentTime, 40, "%Y%m%d%H%M%S", localTime);
    string currentTimeString(currentTime);

    return currentTimeString;

}

/**
 * Ground Station mode.
 * 
 * @return 0: IDLE, 1: MENU, 2: LAIRD_SCREEN, 10: REMOTE_CONTROLLED
 */
int Fotokite::getGroundMode() {
    return state->groundMode;
}

/**
 * Ground Station on time in seconds.
 * 
 * @return 
 */
double Fotokite::getRuntimeS() {
    return state->runtimeS;
}

/**
 * Ground Station battery voltage in volts.
 * 
 * @return 
 */
double Fotokite::getGroundBattVoltage() {
    return state->groundBattVoltage;
}

/**
 * Tether length in encoder counts relative to the when the Ground Station was turned on.
 * 
 * @return 
 */
double Fotokite::getRelTetherLength() {
    return state->relTetherLength;
}

/**
 * QX, QY, QZ, QW are forming a quaternion representing the orientation of the vehicle with respect to its initialization frame.
 * 
 * @return 
 */
double Fotokite::getQX() {
    return state->QX;
}

/**
 * QX, QY, QZ, QW are forming a quaternion representing the orientation of the vehicle with respect to its initialization frame.
 * 
 * @return 
 */
double Fotokite::getQY() {
    return state->QY;
}

/**
 * QX, QY, QZ, QW are forming a quaternion representing the orientation of the vehicle with respect to its initialization frame.
 * 
 * @return 
 */
double Fotokite::getQZ() {
    return state->QZ;
}

/**
 * QX, QY, QZ, QW are forming a quaternion representing the orientation of the vehicle with respect to its initialization frame.
 * 
 * @return 
 */
double Fotokite::getQW() {
    return state->QW;
}

/**
 * Vertical angle of tether in radians where 0 is aligned with gravity.
 * 
 * @return 
 */
double Fotokite::getElevation() {
    return state->elevation;
}

/**
 * Relative horizontal angle of tether in radians w.r.t the vehicle initialization frame.
 * 
 * @return 
 */
double Fotokite::getRelAzimuth() {
    return state->relAzimuth;
}

/**
 * Barometric altitude.
 * 
 * @return 
 */
double Fotokite::getBaroAlt() {
    return state->baroAlt;
}

/**
 * Flight mode.
 * 
 * @return 1: IDLE, 2: ANGLE, 3: ANGLE_RATE, 5: MOTOR_LOW_LEVEL, 48: EMERGENCY_LAND, 49: MOTOR_STOP_RESTART, 50: TIMEOUT, 52: MOTOR_STOP_DISABLE, 53: BATTERY_TOO_LOW_TO_START, 54: NO_TETHER_POWER, 55: HARD_KILL
 */
int Fotokite::getFlightMode() {
    return state->flightMode;
}

/**
 * Flight Unit on time in seconds.
 * 
 * @return 
 */
double Fotokite::getOnTime() {
    return state->onTime;
}

/**
 * Backup battery voltage in volts.
 * @return 
 */
double Fotokite::getBackup() {
    return state->backup;
}

/**
 * Bit fields for the flight flags.
 * 
 * Bit 0: MISSED_TICK
 * Bit 1: I2C_ERROR
 * Bit 2: CRIT_LOW_BATT
 * Bit 3: NO_SYNC_CMDS
 * Bit 4: LOW_LAUNCH_ANGLE
 * Bit 5: MOTOR_ERROR
 * Bit 6: LOW_BATT
 * Bit 7: LOW_BACKUP_BATT
 * Bit 8: EMERGENCY_LAND
 * Bit 9: NO_TETHER_DETECTED
 * Bit 10: HARD_KILL
 * Bit 11: OPTOFORCE_TEMP_INVALID
 * Bit 12: FLYAWAY_CANDIDATE
 * 
 * @return 
 */
unsigned int Fotokite::getFlags() {
    return state->flags;
}

/**
 * Check if the value lies within [-limit, limit] interval.
 * 
 * @param value
 * @param limit
 */
double Fotokite::correctInputValues(double value, double limit) {

    // Check if the values are legal
    if (value > limit) {
        return limit;
    } else if (value < -limit) {
        return -limit;
    } else {
        return value;
    }

}

/**
 * Sends string command to Fotokite
 * 
 * @param command
 */
void Fotokite::sendCommand(string command) {

    this->communication->send(command + '\n');

}

/**
 * Adjusts desired gimbal angle rate for both pitch and roll. Floating point values in rad/s.
 * 
 * @param pitchRate
 * @param rollRate
 */
void Fotokite::gimbal(double pitchRate, double rollRate) {

    // Check if the values are legal
    pitchRate = correctInputValues(pitchRate, 0.3);
    rollRate = correctInputValues(rollRate, 0.3);

    // Send command
    sendCommand("Gimbal " + to_string(pitchRate) + "," + to_string(rollRate));

}

/**
 * Adjusts desired gimbal angle rate for roll. Floating point values in rad/s.
 * 
 * @param pitchRate
 * @param rollRate
 */
void Fotokite::gimbalRoll(double rollRate) {

    // Check if the values are legal
    rollRate = correctInputValues(rollRate, 0.3);

    // Send command
    sendCommand("GimbalRoll " + to_string(rollRate));

}

/**
 * Adjusts desired gimbal angle rate for pitch. Floating point values in rad/s.
 * 
 * @param pitchRate
 * @param rollRate
 */
void Fotokite::gimbalPitch(double pitchRate) {

    // Check if the values are legal
    pitchRate = correctInputValues(pitchRate, 0.3);

    // Send command
    sendCommand("GimbalPitch " + to_string(pitchRate));

}

/**
 * Adjust the position of the Flight Unit by commanding a elevation or azimuth angle rate or a change in tether length. Elevation and azimuth are signed floating point values in rad/s. Length is in spool encoder counts.
 * 
 * @param elevRate
 * @param azimuthRate
 * @param lengthDelta
 */
void Fotokite::pos(double elevRate, double azimuthRate, double lengthDelta) {

    // Check if the values are legal
    elevRate = correctInputValues(elevRate, 0.25);
    azimuthRate = correctInputValues(azimuthRate, 0.2);

    // Send command
    sendCommand("Pos " + to_string(elevRate) + "," + to_string(azimuthRate) + "," + to_string(lengthDelta));

}

/**
 * Adjust the position of the Flight Unit by commanding a elevation. Elevation is signed floating point value in rad/s.
 * 
 * @param elevRate
 */
void Fotokite::posV(double elevRate) {

    // Check if the values are legal
    elevRate = correctInputValues(elevRate, 0.25);

    // Send command
    sendCommand("PosV " + to_string(elevRate));

}

/**
 * Adjust the position of the Flight Unit by commanding an azimuth angle rate. Azimuth is signed floating point values in rad/s.
 * 
 * @param azimuthRate
 */
void Fotokite::posH(double azimuthRate) {

    // Check if the values are legal
    azimuthRate = correctInputValues(azimuthRate, 0.2);

    // Send command
    sendCommand("PosH " + to_string(azimuthRate));

}

/**
 * Adjust the position of the Flight Unit by commanding a change in tether length. Length is in spool encoder counts.
 * 
 * @param lengthDelta
 */
void Fotokite::posL(double lengthDelta) {

    // Check if the values are legal
    lengthDelta = correctInputValues(lengthDelta, numeric_limits<float>::max());

    // Send command
    sendCommand("PosL " + to_string(lengthDelta));

}

/**
 * Adjust the yaw orientation of the Flight Unit by commanding a yaw angle rate, signed floating point, in rad/s.
 * 
 * @param yawRate
 */
void Fotokite::yaw(double yawRate) {

    // Check if the values are legal
    yawRate = correctInputValues(yawRate, 0.4);

    // Send command
    sendCommand("Yaw " + to_string(yawRate));

}

/**
 * Prints the current state of Fotokite to the console.
 * 
 */
void Fotokite::printState() {

    string GSStatus = "!GSStatus " + to_string(getGroundMode()) + "," + to_string(getRuntimeS()) + "," + to_string(getGroundBattVoltage()) + "," + to_string(getRelTetherLength());
    string Attitude = "!Attitude " + to_string(getQX()) + "," + to_string(getQY()) + "," + to_string(getQZ()) + "," + to_string(getQW());
    string Pos = "!Pos " + to_string(getElevation()) + "," + to_string(getRelAzimuth()) + "," + to_string(getBaroAlt());
    string FlightStatus = "!FlightStatus " + to_string(getFlightMode()) + "," + to_string(getOnTime()) + "," + to_string(getBackup()) + "," + to_string(getFlags());

    string currentStatus = GSStatus + "\n" + Attitude + "\n" + Pos + "\n" + FlightStatus;

    cout << currentStatus << endl;

}

/**
 * Returns current tether length in metric units (meters).
 * 
 * @return Tether length in meters
 */
double Fotokite::getScaledTetherLength() {

    // Tether unit transform. 100 ticks is 0.406 m. Therefore, 1 tick is 0.406/100 m.
    double tetherScale = 0.406 / 100;

    return Fotokite::getRelTetherLength() * tetherScale;

}

/**
 * Controls the tether length to specified value.
 * 
 * @param targetAzimuth Desired value
 * @param tolerance Tolerance
 */
double Fotokite::tetherControl(double targetTetherLength, double tolerance) {

    // Current scaled tether length
    double currentScaledTetherLength = Fotokite::getScaledTetherLength();

    // Tether control
    double tetherControl;

    if (targetTetherLength < currentScaledTetherLength - tolerance) {

        // Decrease tether length
        tetherControl = -3;

        posL(tetherControl);

    } else if (targetTetherLength > currentScaledTetherLength + tolerance) {

        // Increase tether length
        tetherControl = 3;

        posL(tetherControl);

    } else {

        // No tether length change necessary
        tetherControl = 0;

        posL(tetherControl);

    }

    return tetherControl;

}

/**
 * Controls the elevation to specified value.
 * 
 * @param targetAzimuth Desired value
 * @param tolerance Tolerance
 */
double Fotokite::elevationControl(double targetElevation, double tolerance) {

    // Current elevation
    double currentElevation = 1.57 - getElevation();

    // Gain of P element of PID
    double pGain = 0.25;

    // Elevation rate
    double elevationRate;

    if (abs(targetElevation - currentElevation) > tolerance) {

        elevationRate = pGain * (targetElevation - currentElevation);
        posV(elevationRate);

    } else {

        elevationRate = 0;
        posV(elevationRate);

    }

    return elevationRate;

}

/**
 * Controls the azimuth to specified value.
 * 
 * @param targetAzimuth Desired value
 * @param tolerance Tolerance
 */
double Fotokite::azimuthControl(double targetAzimuth, double tolerance) {

    // Current azimuth
    double currentAzimuth = getRelAzimuth();

    // Gain of P element of PID
    double pGain = 0.2;

    // Azimuth rate
    double azimuthRate;

    if (abs(targetAzimuth - currentAzimuth) > tolerance) {

        azimuthRate = pGain * (targetAzimuth - currentAzimuth);
        posH(azimuthRate);

    } else {

        azimuthRate = 0;
        posH(azimuthRate);

    }

    return azimuthRate;

}

/**
 * Velocity control for Fotokite's attitude.
 * 
 * @param x Desired X coordinate
 * @param y Desired Y coordinate
 * @param z Desired Z coordinate
 * @param currentTetherLength
 * @param currentElevation
 * @param currentAzimuth
 * @param currentX
 * @param currentY
 * @param currentZ
 */
void Fotokite::velocityControl(double x, double y, double z, double currentTetherLength, double currentElevation, double currentAzimuth, double currentX, double currentY, double currentZ) {

    // Compute Jacobian
    Mat jacobian = Mat::zeros(3, 3, CV_64F);
    jacobian.at<double>(0, 0) = cos(currentElevation) * cos(currentAzimuth);
    jacobian.at<double>(0, 1) = -currentTetherLength * cos(currentAzimuth) * sin(currentElevation);
    jacobian.at<double>(0, 2) = -currentTetherLength * cos(currentElevation) * sin(currentAzimuth);
    jacobian.at<double>(1, 0) = sin(currentElevation);
    jacobian.at<double>(1, 1) = currentTetherLength * cos(currentElevation);
    jacobian.at<double>(1, 2) = 0;
    jacobian.at<double>(2, 0) = -cos(currentElevation) * sin(currentAzimuth);
    jacobian.at<double>(2, 1) = currentTetherLength * sin(currentAzimuth) * sin(currentElevation);
    jacobian.at<double>(2, 2) = -currentTetherLength * cos(currentElevation) * cos(currentAzimuth);

    // Velocity vector going from start to target
    Mat velocityVector = Mat::zeros(3, 1, CV_64F);
    velocityVector.at<double>(0, 0) = x - currentX;
    velocityVector.at<double>(1, 0) = y - currentY;
    velocityVector.at<double>(2, 0) = z - currentZ;

    // Commands for velocity control
    Mat commands = jacobian.inv() * velocityVector;

    // Gain
    double gain = 0.5;
    commands = gain * commands;

    // Send commands
    posL(commands.at<double>(0, 0));
    posV(commands.at<double>(1, 0));
    posH(commands.at<double>(2, 0));
}

/**
 * Position control for Fotokite's attitude.
 * 
 * @param x Desired X coordinate
 * @param y Desired Y coordinate
 * @param z Desired Z coordinate
 * @param currentTetherLength
 * @param currentElevation
 * @param currentAzimuth
 * @param currentX
 * @param currentY
 * @param currentZ
 */
void Fotokite::positionControl(double targetTetherLength, double targetElevation, double targetAzimuth, double tetherTolerance, double ElevationTolerance, double azimuthTolerance, double tetherRate, double elevationRate, double azimuthRate) {

    // Tether control
    tetherRate = tetherControl(targetTetherLength, tetherTolerance);

    // Elevation control
    elevationRate = elevationControl(targetElevation, ElevationTolerance);

    // Azimuth control
    azimuthRate = azimuthControl(targetAzimuth, azimuthTolerance);

}

/**
 * Computes length of the portion of the tether between the last contact point
 * and the ground control station. This portion of the tether is static and does
 * not move.
 * 
 * @param newContactPointX X coordinate of the new contact point
 * @param newContactPointY Y coordinate of the new contact point
 * @param newContactPointZ Z coordinate of the new contact point
 * @return static tether length
 */
struct Fotokite::TetherLength Fotokite::computeStaticTetherLength(double newContactPointX, double newContactPointY, double newContactPointZ) {
    
    if (contactPoints.empty()) {
        
        // There are no contact points
        TetherLength tetherLength;
        tetherLength.segment = 0;
        tetherLength.total = 0;
        return tetherLength;
        
    } else {
        
        // Get last contact point
        ContactPoint lastContactPoint = contactPoints.top();
        
        // Compute segment length and total length
        double newSegmentTetherLength = sqrt(pow(newContactPointX - lastContactPoint.x, 2) + pow(newContactPointY - lastContactPoint.y, 2) + pow(newContactPointZ - lastContactPoint.z, 2));
        double newTotalTetherLength = lastContactPoint.totalTetherLength + newSegmentTetherLength;

        TetherLength tetherLength;
        tetherLength.segment = newSegmentTetherLength;
        tetherLength.total = newTotalTetherLength;
        return tetherLength;
        
    }
    
}

/**
 * Add new contact point to the list of all active contact points.
 * 
 * @param newContactPointX
 * @param newContactPointY
 * @param newContactPointZ
 */
void Fotokite::addContactPoint(double newContactPointX, double newContactPointY, double newContactPointZ) {
    
    // Compute static tether length
    TetherLength tetherLength = computeStaticTetherLength(newContactPointX, newContactPointY, newContactPointZ);  
    
    // Initialize new contact point
    ContactPoint newContactPoint = ContactPoint(newContactPointX, newContactPointY, newContactPointZ, tetherLength.segment, tetherLength.total);

    // Push it on the stack
    contactPoints.push(newContactPoint);
    
}

/**
 * Update list of active contact points with the new contact point. It might be
 * necessary to add the new contact point or relax the previous one.
 * 
 * @param newContactPointX
 * @param newContactPointY
 * @param newContactPointZ
 */
void Fotokite::updateContactPoints(double newContactPointX, double newContactPointY, double newContactPointZ) {

    // Stack is empty
    if (contactPoints.empty()) {
        
        addContactPoint(newContactPointX, newContactPointY, newContactPointZ);
                
        return;

    }

    ContactPoint lastContactPoint = contactPoints.top();

    if (!(newContactPointX == lastContactPoint.x && newContactPointY == lastContactPoint.y && newContactPointZ == lastContactPoint.z)) {

        // Stack has only one element, therefore add the new contact point
        if (contactPoints.size() == 1) {

            addContactPoint(newContactPointX, newContactPointY, newContactPointZ);

            return;

        }

        ContactPoint lastContactPointTemporary = contactPoints.top();
        contactPoints.pop();
        ContactPoint secondToLastContactPoint = contactPoints.top();
        contactPoints.push(lastContactPointTemporary);

        if (newContactPointX == secondToLastContactPoint.x && newContactPointY == secondToLastContactPoint.y && newContactPointZ == secondToLastContactPoint.z) {

            contactPoints.pop();

        } else {

            addContactPoint(newContactPointX, newContactPointY, newContactPointZ);

        }

    }

}

/**
 * Go to specified waypoint given in Cartesian coordinate system.
 * 
 * @param x
 * @param y
 * @param z
 * @param theta_x Not implemented
 * @param theta_y Not implemented
 * @param theta_z Not implemented
 */
void Fotokite::goToWaypoint(double targetX, double targetY, double targetZ, double thetaX, double thetaY, double thetaZ, double contactPointX, double contactPointY, double contactPointZ) {

    updateContactPoints(contactPointX, contactPointY, contactPointZ);
    
    // Print stack for debugging
//    ContactPoint cp = contactPoints.top();
//    cout << cp.x << " " << cp.y << " " << cp.z << " " << cp.segmentTetherLength << " " << cp.totalTetherLength << endl;
//    if (contactPoints.size() > 1) {
//        contactPoints.pop();
//        ContactPoint cp2 = contactPoints.top();
//        cout << cp2.x << " " << cp2.y << " " << cp2.z << " " << cp2.segmentTetherLength << " " << cp2.totalTetherLength << endl;
//        contactPoints.push(cp);
//    }
//    cout << endl;
//    
//    return;

    // Waypoint acceptance radius (0.4 m is approximate position error of Fotokite)
    double waypointAcceptanceRadius = 0.4;

    // Target tether, elevation, and azimuth
    double targetTetherLength;
    double targetElevation;
    double targetAzimuth;

    //    if (contactPointX == 0 && contactPointY == 0 && contactPointZ == 0) { // TODO remove this
    //
    //        // Tether is not touching any obstacle
    //        targetTetherLength = sqrt(targetX * targetX + targetY * targetY + targetZ * targetZ);
    //        targetElevation = asin(targetY / targetTetherLength);
    //        targetAzimuth = atan2(targetX, targetZ) - PI / 2;
    //
    //    } else {

    // Tether is touching an obstacle at contact point
    targetAzimuth = atan2(-(targetZ - contactPointZ), targetX - contactPointX);
    targetElevation = asin((targetY - contactPointY) / (sqrt(pow(targetX - contactPointX, 2) + pow(targetY - contactPointY, 2) + pow(targetZ - contactPointZ, 2))));
    targetTetherLength = contactPoints.top().totalTetherLength + sqrt(pow(targetX - contactPointX, 2) + pow(targetY - contactPointY, 2) + pow(targetZ - contactPointZ, 2));

    //    }
    
    // Fotokite can only reach elevation angle of 0.5, so ignore all the waypoints bellow 0.5 elevation angle
    if (targetElevation < 0.5) {
        
        // Print waypoint abandoned
        cout << "Waypoint " << targetX << " " << targetY << " " << targetZ << " abandoned (too low)." << endl;
        
        // Return
        return;
        
    }

    // Was waypoint reached
    bool waypointReached = false;

    // Current tether, elevation, and azimuth
    double currentTetherLength;
    double currentElevation;
    double currentAzimuth;

    // Current x, y, z
    double currentX;
    double currentY;
    double currentZ;

    // Distance to waypoint
    double waypointDistance;

    // Control tolerance
    double tetherTolerance = 0;
    double ElevationTolerance = 0;
    double azimuthTolerance = 0;

    // Control
    double tetherRate;
    double elevationRate;
    double azimuthRate;

    // While waypoint is not reached
    while (!waypointReached) {

        // Current tether, elevation, and azimuth
        currentTetherLength = getScaledTetherLength();
        currentElevation = 1.57 - getElevation();
        currentAzimuth = getRelAzimuth();

        // Correct target azimuth. 180° and -180° is the same. Therefore, if the difference is too high, bring it back.
        if (abs(currentAzimuth - targetAzimuth) > PI) {

            if (currentAzimuth < targetAzimuth) {

                targetAzimuth = targetAzimuth - 2 * PI;

            } else {

                targetAzimuth = targetAzimuth + 2 * PI;

            }
        }

        // Current x, y, z
        currentX = (currentTetherLength - contactPoints.top().totalTetherLength) * cos(currentElevation) * cos(currentAzimuth) + contactPointX;
        currentY = (currentTetherLength - contactPoints.top().totalTetherLength) * sin(currentElevation) + contactPointY;
        currentZ = -(currentTetherLength - contactPoints.top().totalTetherLength) * cos(currentElevation) * sin(currentAzimuth) + contactPointZ;

#ifdef VELOCITY_CONTROL
        velocityControl(targetX, targetY, targetZ, currentTetherLength, currentElevation, currentAzimuth, currentX, currentY, currentZ);
#else
        positionControl(targetTetherLength, targetElevation, targetAzimuth, tetherTolerance, ElevationTolerance, azimuthTolerance, tetherRate, elevationRate, azimuthRate);
#endif

        // Distance to waypoint
        waypointDistance = sqrt(pow(currentX - targetX, 2) + pow(currentY - targetY, 2) + pow(currentZ - targetZ, 2));

        // Check if waypoint was reached
        waypointReached = waypointDistance < waypointAcceptanceRadius;

        // Log
        log(targetX, targetY, targetZ, targetTetherLength, targetElevation, targetAzimuth, currentTetherLength, currentElevation, currentAzimuth, currentX, currentY, currentZ, contactPointX, contactPointY, contactPointZ, tetherRate, elevationRate, azimuthRate, waypointAcceptanceRadius, waypointDistance, waypointReached);

        // Print debugging information to console
        cout << targetTetherLength << " " << currentTetherLength << " | " << targetElevation << " " << currentElevation << " | " << targetAzimuth << " " << currentAzimuth << " | " << waypointDistance << " | " << contactPointX << " " << contactPointY << " " << contactPointZ << " | " << contactPoints.top().totalTetherLength << endl;
        //cout << currentElevation << " " << currentAzimuth << endl;      
    }

    // Print waypoint reached
    cout << "Waypoint " << targetX << " " << targetY << " " << targetZ << " reached." << endl;
}

/**
 * Scale waypoints as planner coordinates are in obstacle units.
 * 
 * @param x
 * @param y
 * @param z
 * @param contactPointX
 * @param contactPointY
 * @param contactPointZ
 * @param scale
 */
void Fotokite::scaleCoordinates(double& x, double& y, double& z, double& contactPointX, double& contactPointY, double& contactPointZ, double scale) {

    // To convert coordinates to meters (used by Fotokite), multiply each coordinate by scale.
    x *= scale;
    y *= scale;
    z *= scale;
    contactPointX *= scale;
    contactPointY *= scale;
    contactPointZ *= scale;

}

/**
 * Rearrange the planner coordinates to correspond to Fotokite coordinates.
 * 
 * @param x
 * @param y
 * @param z
 * @param contactPointX
 * @param contactPointY
 * @param contactPointZ
 */
void Fotokite::rearrangeCoordinates(double& x, double& y, double& z, double& contactPointX, double& contactPointY, double& contactPointZ) {
    double oldX = x;
    double oldY = y;
    double oldZ = z;
    x = oldY;
    y = oldZ;
    z = oldX;
    double oldContactPointX = contactPointX;
    double oldContactPointY = contactPointY;
    double oldContactPointZ = contactPointZ;
    contactPointX = oldContactPointY;
    contactPointY = oldContactPointZ;
    contactPointZ = oldContactPointX;
}

/**
 * Executes path specified in the given waypoint file. The waypoint file has one
 * waypoint per line. Each line contain x, y, and z separated by space.
 * 
 * @param fileName Name of waypoint file
 */
void Fotokite::executePath(string fileName) {
    
    // Waypoint file
    ifstream file(fileName);

    // Waypoint file line
    string line;

    // Open waypoint file
    if (file.is_open()) {

        // For each line (one line is one waypoint)
        while (getline(file, line)) {

            string::size_type nextCharacterPosition;

            // Parse x
            double x = stod(line, &nextCharacterPosition);

            // Parse y
            line = line.substr(nextCharacterPosition);
            double y = stod(line, &nextCharacterPosition);

            // Parse z
            line = line.substr(nextCharacterPosition);
            double z = stod(line, &nextCharacterPosition);

            // Parse contact point x
            line = line.substr(nextCharacterPosition);
            double contactPointX = stod(line, &nextCharacterPosition);

            // Parse contact point y
            line = line.substr(nextCharacterPosition);
            double contactPointY = stod(line, &nextCharacterPosition);

            // Parse contact point z
            line = line.substr(nextCharacterPosition);
            double contactPointZ = stod(line, &nextCharacterPosition);

            // Scale planner coordinates (obstacle units) to Fotokite coordinates (meters). One planner unit is 33 cm.
            scaleCoordinates(x, y, z, contactPointX, contactPointY, contactPointZ, 0.33);

            // Rearrange the planner coordinates to correspond to Fotokite coordinates
            rearrangeCoordinates(x, y, z, contactPointX, contactPointY, contactPointZ);

            // Shift the entire path up to prevent beeing too close to the ground as Fotokite has lower limit on elevation angle (TODO fix this)
            //            double offset = 1;
            //            y += offset;
            //            contactPointY += offset; // TODO offset to contact point

            // Print target waypoint
//            cout << "Going to waypoint " << x << " " << y << " " << z << " with contact point " << contactPointX << " " << contactPointY << " " << contactPointZ << "." << endl;

            // Go to waypoint
            goToWaypoint(x, y, z, 0, 0, 0, contactPointX, contactPointY, contactPointZ);
            
        }

        // Close waypoint file
        file.close();

    } else {

        cout << "Error opening waypoint file " << fileName << "." << endl;

    }

    cout << "Path completed." << endl;

}

/**
 * Log data.
 * 
 * @param targetX
 * @param targetY
 * @param targetZ
 * @param targetTetherLength
 * @param targetElevation
 * @param targetAzimuth
 * @param currentTetherLength
 * @param currentElevation
 * @param currentAzimuth
 * @param currentX
 * @param currentY
 * @param currentZ
 * @param contactPointX
 * @param contactPointY
 * @param contactPointZ
 * @param tetherRate
 * @param elevationRate
 * @param azimuthRate
 * @param waypointAcceptanceRadius
 * @param waypointDistance
 * @param waypointReached
 */
void Fotokite::log(double targetX, double targetY, double targetZ, double targetTetherLength, double targetElevation, double targetAzimuth, double currentTetherLength, double currentElevation, double currentAzimuth, double currentX, double currentY, double currentZ, double contactPointX, double contactPointY, double contactPointZ, double tetherRate, double elevationRate, double azimuthRate, double waypointAcceptanceRadius, double waypointDistance, double waypointReached) {

    // Log time
    logFile << getCurrentTime();
    logFile << " ";

    logFile << targetX;
    logFile << " ";

    logFile << targetY;
    logFile << " ";

    logFile << targetZ;
    logFile << " ";

    logFile << currentX;
    logFile << " ";

    logFile << currentY;
    logFile << " ";

    logFile << currentZ;
    logFile << " ";

    logFile << targetTetherLength;
    logFile << " ";

    logFile << targetElevation;
    logFile << " ";

    logFile << targetAzimuth;
    logFile << " ";

    logFile << currentTetherLength;
    logFile << " ";

    logFile << currentElevation;
    logFile << " ";

    logFile << currentAzimuth;
    logFile << " ";
    
    logFile << contactPointX;
    logFile << " ";

    logFile << contactPointY;
    logFile << " ";

    logFile << contactPointZ;
    logFile << " ";

    logFile << tetherRate;
    logFile << " ";

    logFile << elevationRate;
    logFile << " ";

    logFile << azimuthRate;
    logFile << " ";

    logFile << waypointAcceptanceRadius;
    logFile << " ";

    logFile << waypointDistance;
    logFile << " ";

    logFile << waypointReached;
    logFile << " ";

    logFile << "\n";

}