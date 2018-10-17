/* 
 * File:   main.cpp
 * Author: Jan Dufek
 * 
 * Created on April 26, 2017, 6:39 PM
 */

#include "Fotokite.hpp" 

/**
 * This main function contains example usage of the library.
 */
int main(int argc, char *argv[]) {

    // Initialize Fotokite interface for OCU server
//    Fotokite * fotokite = new Fotokite("127.0.0.1", 8080, 8080);

    // Initialize Fotokite interface for USB serial interface
    Fotokite * fotokite = new Fotokite("/dev/cu.usbmodem1");

    fotokite->takeoff();
    
    // Execute path
    fotokite->executePath("input/velocity_control_test.txt");
    
    fotokite->land();
    
//    fotokite->stopMotors();
    
//    sleep(5);
//    fotokite->takeoff();
//    fotokite->land();
    
    // Go to waypoint
//    fotokite->goToWaypoint(0, 1, 0, 10, 1, 0, 0, 0, 0);
      
    // Elevation angle tether arc correction experiment. For each point the first
    // trial is with the uncorrected elevation angle and the second trial is with
    // the corrected elevation angle for tether arc. All the points are on the
    // diagonal going from the origin at -45° angle from x-axis. They vary in
    // distance from the origin and high. Since they are on the diagonal, x and
    // z coordinates will be the same for a single waypoint.
    
    // Flight altitude 0.5 m
//    fotokite->goToWaypoint(0.5 / sqrt(2), 0.5, 0.5 / sqrt(2), 0, 0, 0, 0, 0, 0);
//    fotokite->goToWaypoint(1 / sqrt(2), 0.5, 1 / sqrt(2), 0, 0, 0, 0, 0, 0);
//    fotokite->goToWaypoint(1.5 / sqrt(2), 0.5, 1.5 / sqrt(2), 0, 0, 0, 0, 0, 0);
//    fotokite->goToWaypoint(2 / sqrt(2), 0.5, 2 / sqrt(2), 0, 0, 0, 0, 0, 0);
//    fotokite->goToWaypoint(2.5 / sqrt(2), 0.5, 2.5 / sqrt(2), 0, 0, 0, 0, 0, 0);
//    fotokite->goToWaypoint(3 / sqrt(2), 0.5, 3 / sqrt(2), 0, 0, 0, 0, 0, 0);
    
    // Flight altitude 1 m
//    fotokite->goToWaypoint(0.5 / sqrt(2), 1, 0.5 / sqrt(2), 0, 0, 0, 0, 0, 0);
//    fotokite->goToWaypoint(1 / sqrt(2), 1, 1 / sqrt(2), 0, 0, 0, 0, 0, 0);
//    fotokite->goToWaypoint(1.5 / sqrt(2), 1, 1.5 / sqrt(2), 0, 0, 0, 0, 0, 0);
//    fotokite->goToWaypoint(2 / sqrt(2), 1, 2 / sqrt(2), 0, 0, 0, 0, 0, 0);
//    fotokite->goToWaypoint(2.5 / sqrt(2), 1, 2.5 / sqrt(2), 0, 0, 0, 0, 0, 0);
//    fotokite->goToWaypoint(3 / sqrt(2), 1, 3 / sqrt(2), 0, 0, 0, 0, 0, 0);
    
    // Flight altitude 1.5 m
//    fotokite->goToWaypoint(0.5 / sqrt(2), 1.5, 0.5 / sqrt(2), 0, 0, 0, 0, 0, 0);
//    fotokite->goToWaypoint(1 / sqrt(2), 1.5, 1 / sqrt(2), 0, 0, 0, 0, 0, 0);
//    fotokite->goToWaypoint(1.5 / sqrt(2), 1.5, 1.5 / sqrt(2), 0, 0, 0, 0, 0, 0);
//    fotokite->goToWaypoint(2 / sqrt(2), 1.5, 2 / sqrt(2), 0, 0, 0, 0, 0, 0);
//    fotokite->goToWaypoint(2.5 / sqrt(2), 1.5, 2.5 / sqrt(2), 0, 0, 0, 0, 0, 0);
//    fotokite->goToWaypoint(3 / sqrt(2), 1.5, 3 / sqrt(2), 0, 0, 0, 0, 0, 0);
    
    // Flight altitude 2 m (first one and then reverse order because of motion capture height limitation)
//    fotokite->goToWaypoint(0.5 / sqrt(2), 2, 0.5 / sqrt(2), 0, 0, 0, 0, 0, 0);
//    fotokite->goToWaypoint(3 / sqrt(2), 2, 3 / sqrt(2), 0, 0, 0, 0, 0, 0);
//    fotokite->goToWaypoint(2.5 / sqrt(2), 2, 2.5 / sqrt(2), 0, 0, 0, 0, 0, 0);
//    fotokite->goToWaypoint(2 / sqrt(2), 2, 2 / sqrt(2), 0, 0, 0, 0, 0, 0);
//    fotokite->goToWaypoint(1.5 / sqrt(2), 2, 1.5 / sqrt(2), 0, 0, 0, 0, 0, 0);
//    fotokite->goToWaypoint(1 / sqrt(2), 2, 1 / sqrt(2), 0, 0, 0, 0, 0, 0);
    
    // Print state
//    while (true) {
//        fotokite->printState();
//    }
    
    // Print attitude
//    while (true) {
//        cout << "!Attitude " + to_string(fotokite->getQX()) + "," + to_string(fotokite->getQY()) + "," + to_string(fotokite->getQZ()) + "," + to_string(fotokite->getQW()) << endl; 
//    }

    // Ground status message
    //    fotokite->getGroundMode();
    //    fotokite->getRuntimeS();
    //    fotokite->getGroundBattVoltage();
    //    fotokite->getRelTetherLength();

    // Attitude message
    //    fotokite->getQX();
    //    fotokite->getQY();
    //    fotokite->getQZ();
    //    fotokite->getQW();

    // Position message
    //    fotokite->getElevation();
    //    fotokite->getRelAzimuth();
    //    fotokite->getBaroAlt();

    // Flight status message
    //    fotokite->getFlightMode();
    //    fotokite->getOnTime();
    //    fotokite->getBackup();
    //    fotokite->getFlags();

    // Gimbal
    //    fotokite->gimbal(0.30, 0.1);
    //    fotokite->gimbalRoll(0.29);
    //    fotokite->gimbalPitch(0.30);

    // Position
    //    fotokite->pos(-0.25, -0.2, -77836.8758);
    //    fotokite->posV(0.25);
    //    fotokite->posH(0.200000);
    //    fotokite->posL(0);

    // Yaw
    //    fotokite->yaw(-0.1);
         
    // Delete Fotokite object (important for stopping remote control mode on Fotokite)
    delete fotokite;

    // Clean exit
    return 0;

}