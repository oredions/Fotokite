/* 
 * File:   FotokiteState.cpp
 * Author: Jan Dufek
 * 
 * Created on April 27, 2017, 8:42 PM
 */

#include "FotokiteState.hpp"

FotokiteState::FotokiteState() {
}

FotokiteState::FotokiteState(const FotokiteState& orig) {
}

FotokiteState::~FotokiteState() {
}

/**
 * Parses the Fotokite telemetry message bundle and updates Fotokite state by saving the values to the members.
 * This bundle may contain one or more telemetry messages. The code will parse those using new line character and run updateSingle for each message.
 * 
 * @param message
 */
void FotokiteState::update(string messageBundle) {

    // Single message
    string message;
        
    // For each message in the message bundle, update the state based on the message
    while ((message = nextToken(&messageBundle, "\r\n")) != "") {

        updateSingle(message);

    }
}

/**
 * Takes single telemetry message and updates the state based on this message.
 * 
 * @param message
 */
void FotokiteState::updateSingle(string message) {

    // Message header
    string header = nextToken(&message, " ");

    // Decode the type of message according to the header
    if (header.compare("!GSStatus") == 0) {

        // Ground status message
        this->groundMode = stoi(nextToken(&message, ","));
        this->runtimeS = stod(nextToken(&message, ","));
        this->groundBattVoltage = stod(nextToken(&message, ","));
        this->relTetherLength = stod(nextToken(&message, ","));
        
        NEW_GSSSTATUS_MESSAGE = true;

    } else if (header.compare("!Attitude") == 0) {
 
        // Attitude message
        this->QW = stod(nextToken(&message, ","));
        this->QX = stod(nextToken(&message, ","));
        this->QY = stod(nextToken(&message, ","));
        this->QZ = stod(nextToken(&message, ","));
        
        NEW_ATTITUDE_MESSAGE = true;
        
    } else if (header.compare("!Gimbal") == 0) {
        
        // Attitude message
        this->gimbalQW = stod(nextToken(&message, ","));
        this->gimbalQX = stod(nextToken(&message, ","));
        this->gimbalQY = stod(nextToken(&message, ","));
        this->gimbalQZ = stod(nextToken(&message, ","));
        
        NEW_GIMBAL_MESSAGE = true;

    } else if (header.compare("!Pos") == 0) {
        
        // Position message
        this->elevation = stod(nextToken(&message, ","));
        this->relAzimuth = stod(nextToken(&message, ","));
        this->baroAlt = stod(nextToken(&message, ","));
        
        NEW_POS_MESSAGE = true;

    } else if (header.compare("!FlightStatus") == 0) {

        // Flight status message
        this->flightMode = stoi(nextToken(&message, ","));
        this->onTime = stod(nextToken(&message, ","));
        this->backup = stod(nextToken(&message, ","));
        this->flags = stoi(nextToken(&message, ","));

        NEW_FLIGHTSTATUS_MESSAGE = true;
        
    } else if (header.compare("!Param") == 0) {
        
        // This message is not documented in the Fotokite Pro 1 Remote Control Protocol v0.2
        // Just ignore it
        
    } else if (header[0] == '!') {

        // All telemetry packets begin with a ‘!‘ character.

        // We have unknown telemetry message, this might be a problem so throw exception
        string errorMessage = "Unknown telemetry message: " + header;
        
        // Optional: print error for unknown telemetry message
        // cerr << errorMessage << endl;
        // Optional: throw exception for unknown telemetry message
        // throw errorMessage;

    } else {
        // Unknown message from Fotokite, this might still be good, since it might not be a telemetry message
        // cerr << "Error: Unknown message from Fotokite." << endl;
        // cout << message << endl;
    }

}

/**
 * Gets next token from the message based on the specified delimiter.
 * 
 * @param message message to be tokenized
 * @param delimiter delimiter to be used to split the message
 * @return 
 */
string FotokiteState::nextToken(string * message, string delimiter) {

    // Check if the delimiter or message is empty
    if (delimiter.empty() || message->empty()) {
        return "";
    }

    // Find the delimiter position
    size_t delimiter_position = message->find(delimiter);

    // Extract token for the message
    string token = message->substr(0, delimiter_position);

    // If the delimiter was not found, delete the rest of the message, else delete just the token
    if (delimiter_position > message->length()) {

        // Erase the entire message
        message->erase(0, message->length());

    } else {

        // Erase token and delimiter from the message
        message->erase(0, delimiter_position + delimiter.length());

    }

    // Return token
    return token;
}