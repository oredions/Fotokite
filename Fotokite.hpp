/* 
 * File:   Fotokite.hpp
 * Author: Jan Dufek
 *
 * Created on April 27, 2017, 2:20 PM
 */

#ifndef FOTOKITE_HPP
#define FOTOKITE_HPP

#include <stdio.h>
#include <iostream>
#include <fstream>
#include <string>
#include <cstring>
#include <cmath>
#include <limits>

#include "SocketCommunication.hpp"
#include "SerialCommunication.hpp"

using namespace std;

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
    
    void goToWaypoint(double, double, double, double, double, double);
    
    void executePath(string);
    
private:
    
    FotokiteState * state;
    
    Communication * communication;
    
    // Log file
    ofstream logFile;
    
    string getCurrentTime();
    
    void sendCommand(string);
    
    void log(double, double, double, double, double, double, double, double, double, double, double, double, double, double, double, double, double, double);
    
    double correctInputValues(double, double);
    
    double getScaledTetherLenght();
    
    double tetherControl(double, double);
    
    double elevationControl(double, double);
    
    double azimuthControl(double, double);
    
};

#endif /* FOTOKITE_HPP */

