/* 
 * File:   Communication.cpp
 * Author: Jan Dufek
 * 
 * Created on May 2, 2017, 1:05 PM
 */

#include "Communication.hpp"

/**
 * Initialize communication with Fotokite.
 * 
 * @param fotokiteState
 */
Communication::Communication(FotokiteState * fotokiteState) {
    
    // Save Fotokite state object
    this->fotokiteState = fotokiteState;
    
}

Communication::Communication(const Communication& orig) {
}

Communication::~Communication() {
}

/**
 * Starts remote control mode on Fotokite.
 */
void Communication::startRemoteControl() {
    
    // Initialize Fotokite to known state
    send("stop\n");
    
    // Send 5 carriage returns
    send("\n");
    send("\n");
    send("\n");
    send("\n");
    send("\n");
    
    // Increase gains in reel motor control loop to mitigate the dead zone
    send("GSLowControl params 2 2\n");
//    send("GSLowControl params 3 1\n");

    // Start remote control
    send("RemoteControl start\n");
    
    // Disable checksum
    send("Checksum 0\n");

    // Change the attitude and position message frequency to 10 Hz (should send only once but send it every time remote control is started just to be sure)
    send("Pset 117,10.0\n");
    send("Pcommit\n");
    
    // Stop debugging information from being sent from airframe down to the ground station
    send("Pset 119,0\n");
    send("Pcommit\n");
    
    // Set update frequency
    send("Get_GroundStatus 0\n");
    send("Get_Att 0\n");
    send("Get_Pos 0\n");
    send("Get_FlightStatus 0\n");
    send("Get_Gimbal 0\n");
    
}

/**
 * Start listener that will be listening for incoming messages.
 */
void Communication::startListener() {

    listening = true;
    listener= thread(&Communication::listen, this);

}

/**
 * Listen for new message.
 */
void Communication::listen() {
    
    string newMessage;
    
    while (listening) {
        newMessage = receive();
        
        if (! newMessage.empty()) {
            fotokiteState->update(newMessage);
        }
    }
}