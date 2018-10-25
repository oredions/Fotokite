/* 
 * File:   FotokiteState.hpp
 * Author: Jan Dufek
 *
 * Created on April 27, 2017, 8:42 PM
 */

#ifndef FOTOKITESTATE_HPP
#define FOTOKITESTATE_HPP

#include <string>
#include <cstring>
#include <iostream>

using namespace std;

class FotokiteState {
public:
    FotokiteState();
    FotokiteState(const FotokiteState& orig);
    virtual ~FotokiteState();
    
    // Ground status message
    int groundMode; 
    double runtimeS;
    double groundBattVoltage;
    double relTetherLength;
    
    // Attitude message
    double QX;
    double QY;
    double QZ;
    double QW;
    
    // Gimbal message
    double gimbalQX;
    double gimbalQY;
    double gimbalQZ;
    double gimbalQW;
    
    // Position message
    double elevation;
    double relAzimuth;
    double baroAlt;
    
    // Flight status message
    int flightMode;
    double onTime;
    double backup;
    unsigned int flags : 13;
    
    void update(string);
    
    // Check if all messages from Fotokite were received and the system is initialized
    bool NEW_GSSSTATUS_MESSAGE = false;
    bool NEW_ATTITUDE_MESSAGE = false;
    bool NEW_GIMBAL_MESSAGE = false;
    bool NEW_POS_MESSAGE = false;
    bool NEW_FLIGHTSTATUS_MESSAGE = false;
    
    bool LOG_DATA = false;
    
private:
    
    string nextToken(string *, string);
    
    void updateSingle(string);

};

#endif /* FOTOKITESTATE_HPP */

