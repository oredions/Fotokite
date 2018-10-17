/* 
 * File:   mainGoToWaypoint.cpp
 * Author: Jan Dufek
 * 
 * Created on October 16, 2018, 7:52 PM
 */

#include <stdio.h>

#include "Fotokite.hpp" 

/**
 * 
 */
int main(int argc, char *argv[]) {

    // Check if the program has 9 arguments
    if (argc != 10) {
        return -1;
    }
    
    // Initialize Fotokite interface for OCU server
//    Fotokite * fotokite = new Fotokite("127.0.0.1", 8080, 8080);
    Fotokite * fotokite = new Fotokite("/dev/cu.usbmodem1");
    
    // Initialize waypoint
    double waypoint[9];
    
    // Parse the waypoint from the program argument
    for (int i = 1; i < argc; i++) {
        
        // Convert string to double
        if (sscanf(argv[i], "%lg", & waypoint[i - 1]) != 1) {
            return i;
        }
        
    }
    
    // Go to waypoint
    fotokite->goToWaypoint(waypoint);

    // Delete Fotokite object (important for stopping remote control mode on Fotokite)
    delete fotokite;

    // Clean exit
    return 0;

}