#include "Fotokite.hpp" 

int main(int argc, char *argv[]) {

    // Initialize Fotokite interface for OCU server
    //    Fotokite * fotokite = new Fotokite("127.0.0.1", 5050);

    // Initialize Fotokite interface for USB serial interface
    Fotokite * fotokite = new Fotokite("/dev/cu.usbmodem1");

    // Execute path
    fotokite->executePath("input/path.txt");

    // Go to waypoint
    //    fotokite->goToWaypoint(3, 1.5, 3, 0, 0, 0);

    // Print state
    //    while (true) {
    //        fotokite->printState();
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