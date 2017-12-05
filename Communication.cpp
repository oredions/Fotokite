/* 
 * File:   Communication.cpp
 * Author: Jan Dufek
 * 
 * Created on May 2, 2017, 1:05 PM
 */

#include "Communication.hpp"

Communication::Communication(FotokiteState * fotokiteState) {
    
    // Save Fotokite state object
    this->fotokiteState = fotokiteState;
    
}

Communication::Communication(const Communication& orig) {
}

Communication::~Communication() {
}

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

void Communication::startListener() {

    listening = true;
    listener= thread(&Communication::listen, this);

}

void Communication::listen() {
    
    string newMessage;
    
    while (listening) {
        newMessage = receive();
        
        if (! newMessage.empty()) {
            fotokiteState->update(newMessage);
        }
    }
}