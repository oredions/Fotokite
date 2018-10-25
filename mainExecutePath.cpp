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
    if (argc != 1 && argc != 2 ) {
        return -1;
    }
    
    // Initialize Fotokite interface for OCU server
    Fotokite * fotokite = new Fotokite("/tmp/cmds", "127.0.0.1", 8080);
    
    if (argc == 1) {
        
        // Execute waypoint sequence specified on standard input
        fotokite->executePath();
        
    } else {
        
        // Execute waypoint file specified by a path in the argument
        fotokite->executePath(argv[1]);
    
    }

    // Delete Fotokite object (important for stopping remote control mode on Fotokite)
    delete fotokite;

    // Clean exit
    return 0;

}
