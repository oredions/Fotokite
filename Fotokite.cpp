/* 
 * File:   Fotokite.cpp
 * Author: Jan Dufek
 * 
 * Created on April 27, 2017, 2:20 PM
 */

#include "Fotokite.hpp"
#include <iostream>
#include <iomanip>
#include <fstream>
#include <string>

#define PI 3.14159265
#define g 9.80665

// Uncomment if you want to correct the elevation angle for tether arc created by gravity
//#define CORRECT_ELEVATION_ANGLE

// Switch between position control and velocity control. Comment if you want position control. Uncomment if you want velocity control.
//#define VELOCITY_CONTROL

/**
 * Initialize wireless communication with the Fotokite server.
 * 
 * @param ip_address
 * @param port
 */
Fotokite::Fotokite(const char * ip_address, const short port_send, const short port_receive) {

    // Initialize Fotokite state
    state = new FotokiteState();

    // Initialize communication with Fotokite OCU Server
    communication = new SocketCommunication(state, ip_address, port_send, port_receive);

    // Initialize log
    logFile.open("log/" + getCurrentTime() + ".txt");

    // Wait for all the messages to be initialized to make sure Fotokite status makes sense
    while (!(state->NEW_GSSSTATUS_MESSAGE && state->NEW_ATTITUDE_MESSAGE && state->NEW_GIMBAL_MESSAGE && state->NEW_POS_MESSAGE && state->NEW_FLIGHTSTATUS_MESSAGE));

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

    // Wait for all the messages to be initialized to make sure Fotokite status makes sense
    while (!(state->NEW_GSSSTATUS_MESSAGE && state->NEW_ATTITUDE_MESSAGE && state->NEW_GIMBAL_MESSAGE && state->NEW_POS_MESSAGE && state->NEW_FLIGHTSTATUS_MESSAGE));

}

Fotokite::Fotokite(const Fotokite& orig) {
}

/**
 * Stop Fotokite motion, stop communication, and close log.
 */
Fotokite::~Fotokite() {

    // Sleep to make sure the messages get accepted. If you don't sleep, the stop messages might get lost because of the previous messages.
    usleep(500000);

    // Stop motion
    stop();

    // Delete communication
    delete communication;

    // Close log
    logFile.close();

}

void Fotokite::stop() {

    // Stop motion
    gimbal(0, 0);
    pos(0, 0, 0);
    yaw(0);

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
 * QW, QX, QY, QZ are forming a quaternion representing the orientation of the vehicle with respect to its initialization frame.
 * 
 * @return 
 */
double Fotokite::getQX() {
    return state->QX;
}

/**
 * QW, QX, QY, QZ are forming a quaternion representing the orientation of the vehicle with respect to its initialization frame.
 * 
 * @return 
 */
double Fotokite::getQY() {
    return state->QY;
}

/**
 * QW, QX, QY, QZ are forming a quaternion representing the orientation of the vehicle with respect to its initialization frame.
 * 
 * @return 
 */
double Fotokite::getQZ() {
    return state->QZ;
}

/**
 * QW, QX, QY, QZ are forming a quaternion representing the orientation of the vehicle with respect to its initialization frame.
 * 
 * @return 
 */
double Fotokite::getQW() {
    return state->QW;
}

/**
 * QW, QX, QY, QZ are forming a quaternion representing the orientation of the vehicle with respect to its initialization frame.
 * The initialization frame is an orthogonal Cartesian coordinate system with the z axis aligned to gravity and x axis pointed to the front of the gimbal at initialization.
 * 
 * @return 
 */
double Fotokite::getGimbalQX() {
    return state->gimbalQX;
}

/**
 * QW, QX, QY, QZ are forming a quaternion representing the orientation of the vehicle with respect to its initialization frame.
 * The initialization frame is an orthogonal Cartesian coordinate system with the z axis aligned to gravity and x axis pointed to the front of the gimbal at initialization.
 * 
 * @return 
 */
double Fotokite::getGimbalQY() {
    return state->gimbalQY;
}

/**
 * QW, QX, QY, QZ are forming a quaternion representing the orientation of the vehicle with respect to its initialization frame.
 * The initialization frame is an orthogonal Cartesian coordinate system with the z axis aligned to gravity and x axis pointed to the front of the gimbal at initialization.
 * 
 * @return 
 */
double Fotokite::getGimbalQZ() {
    return state->gimbalQZ;
}

/**
 * QW, QX, QY, QZ are forming a quaternion representing the orientation of the vehicle with respect to its initialization frame.
 * The initialization frame is an orthogonal Cartesian coordinate system with the z axis aligned to gravity and x axis pointed to the front of the gimbal at initialization.
 * 
 * @return 
 */
double Fotokite::getGimbalQW() {
    return state->gimbalQW;
}

/**
 * Get yaw angle of the orientation of the vehicle with respect to its
 * initialization frame. Yaw is z-axis rotation.
 * Link: https://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles#Quaternion_to_Euler_Angles_Conversion
 * 
 * @return Yaw
 */
double Fotokite::getYaw() {

    double QX = this->getQX();
    double QY = this->getQY();
    double QZ = this->getQZ();
    double QW = this->getQW();

    double sinY = 2.0 * (QW * QZ + QX * QY);
    double cosY = 1.0 - 2.0 * (QY * QY + QZ * QZ);

    double yaw = atan2(sinY, cosY);

    return yaw;

}

/**
 * Get pitch angle of the orientation of the vehicle with respect to its initialization frame. Pitch is y-axis rotation.
 * 
 * @return Pitch
 */
double Fotokite::getPitch() {

    double QX = this->getQX();
    double QY = this->getQY();
    double QZ = this->getQZ();
    double QW = this->getQW();

    double sinP = 2.0 * (QW * QY - QZ * QX);

    double pitch;

    if (fabs(sinP) >= 1) {

        // Use 90 degrees if out of range
        pitch = copysign(M_PI / 2, sinP);

    } else {

        pitch = asin(sinP);

    }

    return pitch;

}

/**
 * Get roll angle of the orientation of the vehicle with respect to its initialization frame. Roll is x-axis rotation.
 * 
 * @return Roll
 */
double Fotokite::getRoll() {

    double QX = this->getQX();
    double QY = this->getQY();
    double QZ = this->getQZ();
    double QW = this->getQW();

    double sinR = 2.0 * (QW * QX + QY * QZ);
    double cosR = 1.0 - 2.0 * (QX * QX + QY * QY);

    double roll = atan2(sinR, cosR);

    return roll;

}

/**
 * Get yaw angle of the orientation of the gimbal with respect to its
 * initialization frame. Yaw is z-axis rotation.
 * Link: https://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles#Quaternion_to_Euler_Angles_Conversion
 * 
 * @return Yaw
 */
double Fotokite::getGimbalYaw() {

    double QX = this->getGimbalQX();
    double QY = this->getGimbalQY();
    double QZ = this->getGimbalQZ();
    double QW = this->getGimbalQW();

    double sinY = 2.0 * (QW * QZ + QX * QY);
    double cosY = 1.0 - 2.0 * (QY * QY + QZ * QZ);

    double yaw = atan2(sinY, cosY);

    return yaw;

}

/**
 * Get pitch angle of the orientation of the gimbal with respect to its initialization frame. Pitch is y-axis rotation.
 * 
 * @return Pitch
 */
double Fotokite::getGimbalPitch() {

    double QX = this->getGimbalQX();
    double QY = this->getGimbalQY();
    double QZ = this->getGimbalQZ();
    double QW = this->getGimbalQW();

    double sinP = 2.0 * (QW * QY - QZ * QX);

    double pitch;

    if (fabs(sinP) >= 1) {

        // Use 90 degrees if out of range
        pitch = copysign(M_PI / 2.0, sinP);

    } else {

        pitch = asin(sinP);

    }

    return pitch;

}

/**
 * Get roll angle of the orientation of the gimbal with respect to its initialization frame. Roll is x-axis rotation.
 * 
 * @return Roll
 */
double Fotokite::getGimbalRoll() {

    double QX = this->getGimbalQX();
    double QY = this->getGimbalQY();
    double QZ = this->getGimbalQZ();
    double QW = this->getGimbalQW();

    double sinR = 2.0 * (QW * QX + QY * QZ);
    double cosR = 1.0 - 2.0 * (QX * QX + QY * QY);

    double roll = atan2(sinR, cosR);

    return roll;

}

/**
 * Vertical angle of tether in radians where 0 is aligned with gravity.
 * 
 * @return elevation angle
 */
double Fotokite::getElevation() {
    return state->elevation;
}

/**
 * Get elevation angle corrected for gravity of the tether.
 * 
 * This provides more precise elevation angle and enables to get more precise
 * cartesian coordinates.
 * 
 * @return elevation angle corrected for tether gravity
 */
double Fotokite::getCorrectedElevation() {

    // Get sensed elevation
    double sensedElevation = 1.57 - getElevation();

    double nominator = cosh(std::log(tan(sensedElevation) + sqrt(pow(tan(sensedElevation), 2) + 1))) - cosh(0);
    double denominator = std::log(tan(sensedElevation) + sqrt(pow(tan(sensedElevation), 2) + 1));

    return atan(nominator / denominator);

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


void Fotokite::disableTetherController() {
    
    sendCommand("Pset 79,0.0");
    
}

void Fotokite::enableTetherController() {
    
    sendCommand("Pset 79,1.0");
    
}

/**
 * Start Fotokite takeoff. If the vehicle is in IDLE state, this will instruct the vehicle to start the motors and takeoff.
 */
void Fotokite::startMotors() {
    
    sendCommand("StartTakeoff");

}

void Fotokite::takeoff() {
    
    cout << "Takeoff initiated" << endl;
    
    disableTetherController();
    
    startMotors();
    
    goToWaypoint(0, 1, 0, 10, 1, 0, 0, 0, 0);
    
    enableTetherController();
    
    cout << "Takeoff complete" << endl;
    
}

/**
 * Stop Fotokite's motors. If the motors are spinning and the vehicle is not in emergency land, this command will stop the propellers.
 */
void Fotokite::stopMotors() {
    
    for (int i = 0; i < 50; i++) {
        
        // Sleep to make sure the message gets accepted
        usleep(50000);
        
        sendCommand("MotorStop");
        
    }
    
}

void Fotokite::land() {
    
    
    cout << "Landing initiated" << endl;
    
    goToWaypoint(0, 1, 0, 10, 1, 0, 0, 0, 0);
    
    disableTetherController();
    
    while (getScaledTetherLength() > 0.1) {
        
        if (state->NEW_GSSSTATUS_MESSAGE) {
            posL(-1);
//            pos2(0,0,-300);
            
            cout << "Landing" << endl;
            
        }
        
    }
    
    stopMotors();
    
    enableTetherController();
    
    cout << "Landing complete" << endl;
    
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
 * Adjust the position of the Flight Unit by commanding an elevation or azimuth
 * angle rate or a change in tether length. Elevation and azimuth are signed
 * floating point values in rad/s. Length is in spool encoder counts.
 * Negative tether length is always spool in.
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
 * Adjust the position of the Flight Unit by commanding an elevation angle rate,
 * an azimuth angle rate, or a tether length velocity. Elevation and azimuth are
 * signed floating point values in rad/s. Tether length velocity is in
 * encoder counts/s. With the default ground station parameters, you will not
 * feel any motor action with tetherLengthVel less than approximately -500.
 * Negative tether length velocity is always spool in.
 * 
 * By experiments with our Fotokite, the minimum for tetherLengthVel is -300.
 * 
 * @param elevRate
 * @param azimuthRate
 * @param tetherLengthVel
 */
void Fotokite::pos2(double elevRate, double azimuthRate, double tetherLengthVel) {

    // Check if the values are legal
    elevRate = correctInputValues(elevRate, 0.25);
    azimuthRate = correctInputValues(azimuthRate, 0.2);
    tetherLengthVel = correctInputValues(tetherLengthVel, 10000);

    // Send command
    sendCommand("Pos2 " + to_string(elevRate) + "," + to_string(azimuthRate) + "," + to_string(tetherLengthVel));

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

    return Fotokite::getRelTetherLength() * TETHER_SCALE;

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
#ifdef CORRECT_ELEVATION_ANGLE

    // Get elevation angle corrected for tether arc caused by gravity
    double currentElevation = getCorrectedElevation();

#else

    // Get uncorrected elevation as read by the Fotokite's sensor
    double currentElevation = 1.57 - getElevation();

#endif

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
void Fotokite::velocityControl(double x, double y, double z, double currentTetherLength, double currentElevation, double currentAzimuth, double currentX, double currentY, double currentZ, double & tetherLengthVelocity, double & elevationRate, double & azimuthRate, double & speed) {

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

    // Velocity vector deltas
    double deltaX = x - currentX;
    double deltaY = y - currentY;
    double deltaZ = z - currentZ;

    // Velocity vector length
    double velocityVectorLength = sqrt(pow(deltaX, 2) + pow(deltaY, 2) + pow(deltaZ, 2));

    // Velocity unit vector going from start to target
    Mat velocityUnitVector = Mat::zeros(3, 1, CV_64F);
    velocityUnitVector.at<double>(0, 0) = deltaX / velocityVectorLength;
    velocityUnitVector.at<double>(1, 0) = deltaY / velocityVectorLength;
    velocityUnitVector.at<double>(2, 0) = deltaZ / velocityVectorLength;

    // Desired speed in m/s
    speed = 0.1;

    // Velocity vector in metric units (m/s)
    Mat velocityVector = Mat::zeros(3, 1, CV_64F);
    velocityVector = velocityUnitVector * speed;

    // Commands for velocity control
    Mat commands = jacobian.inv() * velocityVector;

    // Tether length velocity in m/s
    double tetherLengthVelocityMetric = commands.at<double>(0, 0);

    // Map tether velocity dead zone to active zone by using two linear functions
    if (tetherLengthVelocityMetric > 0) {
        
        tetherLengthVelocity = (tetherLengthVelocityMetric / TETHER_SCALE) + TETHER_NO_MOTION_RATE + (DEAD_ZONE_WIDTH / 2);
        
    } else if (tetherLengthVelocityMetric < 0) {
        
        tetherLengthVelocity = (tetherLengthVelocityMetric / TETHER_SCALE) + TETHER_NO_MOTION_RATE - (DEAD_ZONE_WIDTH / 2);
        
    } else {
        
        tetherLengthVelocity = TETHER_NO_MOTION_RATE;
        
    }

    // Get control rates for elevation and azimuth
    elevationRate = commands.at<double>(1, 0);
    azimuthRate = commands.at<double>(2, 0);

    // Send commands
    pos2(elevationRate, azimuthRate, tetherLengthVelocity);

    // Debugging message
    //cout << tetherLengthVelocity << " " << elevationRate << " " << azimuthRate << " | " << velocityUnitVector.at<double>(0,0) << " " << velocityUnitVector.at<double>(1,0) << " " << velocityUnitVector.at<double>(2,0) << endl; // << " | " << targetAzimuth << " " << currentAzimuth << " | " << waypointDistance << " | " << contactPointX << " " << contactPointY << " " << contactPointZ << " | " << contactPoints.top().totalTetherLength << endl;

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

bool Fotokite::viewControl(double pointOfInterestX, double pointOfInterestY, double pointOfInterestZ, double currentX, double currentY, double currentZ, double yawTolerance, double gimbalPitchTolerance, double & targetYaw, double & targetGimbalPitch, double & currentYaw, double & currentGimbalPitch, double & yawRate, double & gimbalPitchRate) {

    bool yawReached = yawControl(pointOfInterestX, pointOfInterestZ, currentX, currentZ, yawTolerance, targetYaw, currentYaw, yawRate);
    bool gimbalPitchReached = gimbalPitchControl(pointOfInterestX, pointOfInterestY, pointOfInterestZ, currentX, currentY, currentZ, gimbalPitchTolerance, targetGimbalPitch, currentGimbalPitch, gimbalPitchRate);

    return yawReached && gimbalPitchReached;

}

bool Fotokite::yawControl(double pointOfInterestX, double pointOfInterestZ, double currentX, double currentZ, double tolerance, double & targetYaw, double & currentYaw, double & yawRate) {

    // Target yaw
    targetYaw = -atan2(pointOfInterestZ - currentZ, pointOfInterestX - currentX);

    // Current yaw
    currentYaw = getYaw();

    // Correct target yaw. 180° and -180° is the same. Therefore, if the difference is too high, bring it back.
    if (abs(currentYaw - targetYaw) > PI) {

        if (currentYaw < targetYaw) {

            targetYaw = targetYaw - 2 * PI;

        } else {

            targetYaw = targetYaw + 2 * PI;

        }
    }

    // Gain of P element of PID
    double pGain = 0.4;

    if (abs(targetYaw - currentYaw) > tolerance) {

        // Only do yaw control if we have new attitude message
        if (state->NEW_ATTITUDE_MESSAGE) {

            yawRate = pGain * (targetYaw - currentYaw);

            yaw(yawRate);

            // Reset new message flag
            state->NEW_ATTITUDE_MESSAGE = false;

        }

        // Desired yaw not reached yet
        return false;

    } else {

        // Only do yaw control if we have new attitude message
        if (state->NEW_ATTITUDE_MESSAGE) {

            yawRate = 0;
            yaw(yawRate);

            // Reset new message flag
            state->NEW_ATTITUDE_MESSAGE = false;

        }

        // Desired yaw reached
        return true;

    }

}

/**
 * Gimbal pitch control for Fotokite's gimbal.
 * 
 * 
 * @param pointOfInterestX the point Fotokite should look at
 * @param pointOfInterestY
 * @param pointOfInterestZ
 * @param currentX the current Fotokite's position
 * @param currentY
 * @param currentZ
 * @param tolerance controller tolerance
 */
bool Fotokite::gimbalPitchControl(double pointOfInterestX, double pointOfInterestY, double pointOfInterestZ, double currentX, double currentY, double currentZ, double tolerance, double & targetPitch, double & currentPitch, double & pitchRate) {

    // Compute the distance to point of interest   
    double distanceToPointOfInterest = sqrt(pow((currentX - pointOfInterestX), 2) + pow((currentZ - pointOfInterestZ), 2));

    // Target pitch
    targetPitch = atan((currentY - pointOfInterestY) / distanceToPointOfInterest);

    // Current pitch
    currentPitch = getGimbalPitch();

    // Gain of P element of PID
    double pGain = 0.2;

    if (abs(targetPitch - currentPitch) > tolerance) {

        // Only do pitch control if we have new gimbal message
        if (state->NEW_GIMBAL_MESSAGE) {

            pitchRate = pGain * (targetPitch - currentPitch);
            gimbalPitch(pitchRate);

            // Reset new message flag
            state->NEW_GIMBAL_MESSAGE = false;

        }

        // Desired pitch not reached yet
        return false;

    } else {

        // Only do pitch control if we have new gimbal message
        if (state->NEW_GIMBAL_MESSAGE) {

            pitchRate = 0;
            gimbalPitch(pitchRate);

            // Reset new message flag
            state->NEW_GIMBAL_MESSAGE = false;

        }

        // Desired pitch reached
        return true;

    }

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

///**
// * Get lean angle of airframe. The lean angle is angle of the airframe relative 
// * to the horizontal frame. If the airframes , it is the angle between that plane and the horizontal plane.
// * 
// * @return 
// */
//double Fotokite::getLeanAngle() {
//
//    // Get quaternion (it is normalized). Quaternion uses Tait–Bryan coordinate system.
//    double q1 = this->getQX();
//    double q2 = this->getQY();
//    double q3 = this->getQZ();
//    double q0 = this->getQW();
//
//    // Used the Fotokite SDK's coordinate system which is Tait–Bryan. Link: https://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles#Rotation_matrices
//    Mat R = (Mat_<double>(3, 3) <<
//            1 - 2 * (pow(q2, 2) + pow(q3, 2)), 2 * (q1 * q2 - q0 * q3), 2 * (q0 * q2 + q1 * q3),
//            2 * (q1 * q2 + q0 * q3), 1 - 2 * (pow(q1, 2) + pow(q3, 2)), 2 * (q2 * q3 - q0 * q1),
//            2 * (q1 * q3 - q0 * q2), 2 * (q0 * q1 + q2 * q3), 1 - 2 * (pow(q1, 2) + pow(q2, 2)));
//
//    // Vector pointing up
//    Mat n_v = (Mat_<double>(3, 1) <<
//            0,
//            0,
//            1);
//
//    Mat n_g = R * n_v;
//
//    double x_g = n_g.at<double>(0, 0);
//    double y_g = n_g.at<double>(1, 0);
//    double z_g = n_g.at<double>(2, 0);
//
//    //    cout << "x_g: " << x_g << " | y_g: " << y_g << " | z_g:" << z_g << endl;
//
//    double leanAngle = asin(z_g / sqrt(pow(x_g, 2) + pow(y_g, 2) + pow(z_g, 2)));
//
//    //    cout << leanAngle / PI * 180 << endl;
//
//    return leanAngle;
//
//}
//
//double Fotokite::getTetherTension() {
//
//    // See the paper for the symbol definitions
//
//    double beta = getLeanAngle();
//    double theta = 1.57 - getElevation();
//
//    double G = airframeWeight * g;
//
//    double T = (G * cos(beta)) / (sin(beta) * cos(theta) - tan(theta) * cos(theta) * cos(beta));
//
//
//    double F = G / (sin(beta) - tan(theta) * cos(beta));
//    //cout << "T: " << T << " | F: " << F << " | b:" << beta / PI * 180 << endl;
//
//    return T;
//
//}
//
//void Fotokite::updateTetherEndpoint() {
//
//    double T_1 = getTetherTension();
//    double beta = 1.57 - getElevation();
//    double rho = tetherLinearDensity;
//
//    double deltaX = ((T_1 * cos(beta)) / (rho * g)) * std::log(tan(beta) + sqrt(pow(tan(beta), 2) + 1));
//
//    double T_0 = T_1 * cos(beta);
//
//    double a = T_0 / (0.0061 * g);
//
//    double deltaY = a * cosh(deltaX / a);
//
//    tetherEndpointX = deltaX;
//    tetherEndpointY = deltaY;
//
//    //cout << T_1 << " | " << tetherEndpointX << " | " << tetherEndpointY << endl;
//
//    //cout << "Y/X = " << atan(tetherEndpointY/tetherEndpointX) / PI * 180 << " | Beta = " << beta / PI * 180 << endl;
//
//}

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

void Fotokite::goToWaypoint(double waypoint[9]) {
    
    goToWaypoint(waypoint[0], waypoint[1], waypoint[2], waypoint[3], waypoint[4], waypoint[5], waypoint[6], waypoint[7], waypoint[8]);
    
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
void Fotokite::goToWaypoint(double targetX, double targetY, double targetZ, double pointOfInterestX, double pointOfInterestY, double pointOfInterestZ, double contactPointX, double contactPointY, double contactPointZ) {

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
    //    if (targetElevation < 0.5) {
    //        
    //        // Print waypoint abandoned
    //        cout << "Waypoint " << targetX << " " << targetY << " " << targetZ << " abandoned (too low)." << endl;
    //        
    //        // Return
    //        return;
    //        
    //    }

    // Was waypoint reached
    bool waypointReached = false;

    // Was desired view reached
    bool viewReached = false;

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
    
    // Control method (position control vs velocity control)
    bool controlMethod;
    
    // The following are passed by reference for logging purposes
    double speed;
    double targetYaw;
    double targetGimbalPitch;
    double currentYaw;
    double currentGimbalPitch;
    double yawRate;
    double gimbalPitchRate;

    // While waypoint is not reached
    while (!waypointReached || !viewReached) {

        //        // Update tether endpoint
        //        updateTetherEndpoint();

        // Current tether
        currentTetherLength = getScaledTetherLength();

        // Current azimuth
        currentAzimuth = getRelAzimuth();

        // Current elevation
        #ifdef CORRECT_ELEVATION_ANGLE

                // Get elevation angle corrected for tether arc caused by gravity
                currentElevation = getCorrectedElevation();

        #else

                // Get uncorrected elevation as read by the Fotokite's sensor
                currentElevation = 1.57 - getElevation();

        #endif

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

        // Do control only if a new pos message is available (otherwise we would just send the same value anyways and it could lead to jerking)
        if (state->NEW_POS_MESSAGE) {

            // Distance to waypoint
            waypointDistance = sqrt(pow(currentX - targetX, 2) + pow(currentY - targetY, 2) + pow(currentZ - targetZ, 2));

            // Check if waypoint was reached
            waypointReached = waypointDistance < waypointAcceptanceRadius;

            if (!waypointReached) {

#ifdef VELOCITY_CONTROL

                controlMethod = true;
                
                velocityControl(targetX, targetY, targetZ, currentTetherLength, currentElevation, currentAzimuth, currentX, currentY, currentZ, tetherRate, elevationRate, azimuthRate, speed);

#else
                
                controlMethod = false;

                positionControl(targetTetherLength, targetElevation, targetAzimuth, tetherTolerance, ElevationTolerance, azimuthTolerance, tetherRate, elevationRate, azimuthRate);

#endif

            } else {

                // Stop motion of Fotokite
                pos(0, 0, 0);

            }

            // The pos message was used so reset new message flag
            state->NEW_POS_MESSAGE = false;

        }

        // View control. We have to do view control every time, even if the view was previously reached. Control of Fotokite's position might invalidate the view.
        viewReached = viewControl(pointOfInterestX, pointOfInterestY, pointOfInterestZ, currentX, currentY, currentZ, 0.1, 0.1, targetYaw, targetGimbalPitch, currentYaw, currentGimbalPitch, yawRate, gimbalPitchRate);

        // Only log if there is a new information
        if (state->NEW_POS_MESSAGE || state->NEW_ATTITUDE_MESSAGE || state->NEW_GIMBAL_MESSAGE) {
            
            // Log
            log(targetX, targetY, targetZ, targetTetherLength, targetElevation, targetAzimuth,
                    currentTetherLength, currentElevation, currentAzimuth, currentX,
                    currentY, currentZ, contactPointX, contactPointY, contactPointZ,
                    tetherRate, elevationRate, azimuthRate, waypointAcceptanceRadius,
                    waypointDistance, waypointReached, controlMethod, speed, targetYaw,
                    targetGimbalPitch, currentYaw, currentGimbalPitch, pointOfInterestX,
                    pointOfInterestY, pointOfInterestZ, yawRate, gimbalPitchRate);
            
            // Print status
            cout << fixed << std::setprecision(1) << "Waypoint: (" << targetX << ", " << targetY << ", " << targetZ << ")    POI: (" << pointOfInterestX << ", " << pointOfInterestY << ", " << pointOfInterestZ << ")    Current: (" << currentX << ", " << currentY << ", " << currentZ << ")    Distance: " << waypointDistance << " m" << endl;
                    
        }

        // Print debugging information to console
        //cout << currentX << " " << currentY << " " << currentZ << " " << waypointDistance << endl;
        //        cout << targetTetherLength << " " << currentTetherLength << " | " << targetElevation << " " << currentElevation << " " << 1.57 - getElevation() << " | " << targetAzimuth << " " << (currentAzimuth / PI * 180) << " | " << waypointDistance << " | " << currentX << " " << currentY << " " << currentZ << endl;
        //cout << currentElevation << " " << currentAzimuth << endl;
        
    }

    // Waypoint and view is reached so stop Fotokite
    stop();

    // Print waypoint reached
    cout << "Waypoint (" << targetX << ", " << targetY << ", " << targetZ << ") with POI (" << pointOfInterestX << ", " << pointOfInterestY << ", " << pointOfInterestZ << ") reached." << endl;
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
            
            // Parse point of interest x
            line = line.substr(nextCharacterPosition);
            double pointOfInterestX = stod(line, &nextCharacterPosition);
            
            // Parse point of interest y
            line = line.substr(nextCharacterPosition);
            double pointOfInterestY = stod(line, &nextCharacterPosition);
            
            // Parse point of interest z
            line = line.substr(nextCharacterPosition);
            double pointOfInterestZ = stod(line, &nextCharacterPosition);

            // Parse contact point x
            line = line.substr(nextCharacterPosition);
            double contactPointX = stod(line, &nextCharacterPosition);

            // Parse contact point y
            line = line.substr(nextCharacterPosition);
            double contactPointY = stod(line, &nextCharacterPosition);

            // Parse contact point z
            line = line.substr(nextCharacterPosition);
            double contactPointZ = stod(line, &nextCharacterPosition);

            // Scale planner coordinates (obstacle units) to Fotokite coordinates (meters). One planner unit is 33 cm. Only used with cardboard boxes and MATLAB planner.
            //scaleCoordinates(x, y, z, contactPointX, contactPointY, contactPointZ, 0.33);

            // Rearrange the planner coordinates to correspond to Fotokite coordinates. Only used with MATLAB planner.
            //rearrangeCoordinates(x, y, z, contactPointX, contactPointY, contactPointZ);

            // Shift the entire path up to prevent beeing too close to the ground as Fotokite has lower limit on elevation angle (TODO fix this)
            //            double offset = 1;
            //            y += offset;
            //            contactPointY += offset; // TODO offset to contact point

            // Print target waypoint
//            cout << "Going to waypoint " << x << " " << y << " " << z << " with point of interest " << pointOfInterestX << " " << pointOfInterestY << " " << pointOfInterestZ << " with contact point " << contactPointX << " " << contactPointY << " " << contactPointZ << "." << endl;

            // Go to waypoint
            goToWaypoint(x, y, z, pointOfInterestX, pointOfInterestY, pointOfInterestZ, contactPointX, contactPointY, contactPointZ);

        }

        // Close waypoint file
        file.close();

    } else {

        cout << "Error opening waypoint file " << fileName << "." << endl;

    }

    cout << "Path completed." << endl;

}

/**
 * Executes path received as sequence of waypoints on the standard input.
 * The waypoint sequence has one waypoint per line. Each line contain x, y, z,
 * and point of interest x, y, z, and contact point x, y, z separated by space.
 * Each waypoint is ended by new line. The sequence is ended by end of file.
 * 
 */
void Fotokite::executePath() {

    // X
    double x;

    // Y
    double y;

    // Z
    double z;

    // Point of interest x
    double pointOfInterestX;

    // Point of interest y
    double pointOfInterestY;

    // Point of interest z
    double pointOfInterestZ;

    // Contact point x
    double contactPointX;

    // Contact point y
    double contactPointY;

    // Contact point z
    double contactPointZ;
    
    // For each line (one line is one waypoint)
    while (!cin.eof()) {
        
        // Read from standard in
        cin >> x >> y >> z >> pointOfInterestX >> pointOfInterestY >> pointOfInterestZ >> contactPointX >> contactPointY >> contactPointZ;
        
        // Go to waypoint
        goToWaypoint(x, y, z, pointOfInterestX, pointOfInterestY, pointOfInterestZ, contactPointX, contactPointY, contactPointZ);
//        cout << x << " " << y << " " << z << " " << pointOfInterestX << " " << pointOfInterestY << " " << pointOfInterestZ << " " << contactPointX << " " << contactPointY << " " << contactPointZ << endl;

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
void Fotokite::log(double targetX, double targetY, double targetZ, double targetTetherLength,
        double targetElevation, double targetAzimuth, double currentTetherLength, double currentElevation,
        double currentAzimuth, double currentX, double currentY, double currentZ, double contactPointX,
        double contactPointY, double contactPointZ, double tetherRate, double elevationRate,
        double azimuthRate, double waypointAcceptanceRadius, double waypointDistance,
        double waypointReached, bool controlMethod, double speed, double targetYaw,
        double targetGimbalPitch, double currentYaw, double currentGimbalPitch, double pointOfInterestX,
        double pointOfInterestY, double pointOfInterestZ, double yawRate, double gimbalPitchRate) {

    // Log time
    logFile << getCurrentTime();
    logFile << " ";
    
    logFile << controlMethod;
    logFile << " ";
    
    logFile << speed;
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
    
    logFile << targetYaw;
    logFile << " ";

    logFile << targetGimbalPitch;
    logFile << " ";

    logFile << currentTetherLength;
    logFile << " ";

    logFile << currentElevation;
    logFile << " ";

    logFile << currentAzimuth;
    logFile << " ";
    
    logFile << currentYaw;
    logFile << " ";
    
    logFile << currentGimbalPitch;
    logFile << " ";
    
    logFile << pointOfInterestX;
    logFile << " ";

    logFile << pointOfInterestY;
    logFile << " ";

    logFile << pointOfInterestZ;
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
    
    logFile << yawRate;
    logFile << " ";

    logFile << gimbalPitchRate;
    logFile << " ";
    
    logFile << waypointAcceptanceRadius;
    logFile << " ";

    logFile << waypointDistance;
    logFile << " ";

    logFile << waypointReached;
    logFile << " ";

    logFile << "\n";

}