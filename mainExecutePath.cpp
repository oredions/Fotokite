/* 
 * File:   mainExecutePath.cpp
 * Author: Jan Dufek
 * 
 * Created on October 16, 2018, 8:28 PM
 */

#include <stdio.h>

#include "Fotokite.hpp" 

/**
 * 
 */
int main(int argc, char *argv[]) {

    // Check if the program has 1 argument
    if (argc != 2) {
        return -1;
    }
    
    // Initialize Fotokite interface for OCU server
//    Fotokite * fotokite = new Fotokite("127.0.0.1", 8080, 8080);
    Fotokite * fotokite = new Fotokite("/dev/cu.usbmodem1");
    
    // Go to waypoint
    fotokite->executePath(argv[1]);

    // Delete Fotokite object (important for stopping remote control mode on Fotokite)
    delete fotokite;

    // Clean exit
    return 0;

}