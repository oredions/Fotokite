/* 
 * File:   mainTakeoff.cpp
 * Author: Jan Dufek
 * 
 * Created on October 16, 2018, 7:51 PM
 */

#include "Fotokite.hpp" 

/**
 * 
 */
int main(int argc, char *argv[]) {

    // Initialize Fotokite interface for OCU server
//    Fotokite * fotokite = new Fotokite("127.0.0.1", 8080, 8080);
    Fotokite * fotokite = new Fotokite("/dev/cu.usbmodem01");

    // Land
    fotokite->takeoff();

    // Delete Fotokite object (important for stopping remote control mode on Fotokite)
    delete fotokite;

    // Clean exit
    return 0;

}