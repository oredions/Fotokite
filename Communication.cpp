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
    
    // Send 5 carriage returns
    send("\n");
    send("\n");
    send("\n");
    send("\n");
    send("\n");

    // Start remote control
    send("RemoteControl start\n");
    
    // Disable checksum
    send("Checksum 0\n");

    // Set update frequency
    send("Get_GroundStatus 0\n");
    send("Get_Att 0\n");
    send("Get_Pos 0\n");
    send("Get_FlightStatus 0\n");
    
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