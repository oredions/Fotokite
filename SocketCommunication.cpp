/* 
 * File:   SocketCommunication.cpp
 * Author: Jan Dufek
 */

#include "SocketCommunication.hpp"

SocketCommunication::SocketCommunication(FotokiteState * fotokiteState, const char * ip_address, const short port) : Communication(fotokiteState) {
     
    // Create socket descriptor. We want to use datagram UDP.
    socket_descriptor = socket(AF_INET, SOCK_STREAM, 0);

    if (socket_descriptor < 0) {
        cout << "Error creating socket descriptor." << endl;
    }

    // Create socket address
    socket_address.sin_family = AF_INET;
    socket_address.sin_addr.s_addr = inet_addr(ip_address);
    socket_address.sin_port = htons(port);

    // Connect to the server
    int status = connect(socket_descriptor, (struct sockaddr *) &socket_address, sizeof (socket_address));

    // Check if the connection was established
    if (status < 0) {
        cerr << "Error connecting to the TCP server at IP address " << ip_address << " at port " << to_string(port) << "! Make sure your machine is connected to the OCU server via ethernet and has IP address on the same network." << endl;
    }
    
    // Initialize Fotokite remote control mode
    this->startRemoteControl();
    
    // Start listener
    this->startListener();

}

SocketCommunication::SocketCommunication(const SocketCommunication& orig) : Communication(orig) {
}

SocketCommunication::~SocketCommunication() {
    close_connection();
}

void SocketCommunication::send(string message) {

    // Send message
    int status = write(socket_descriptor, message.data(), message.size());

    // Check if the action was successful
    if (status < 0) {
        cerr << "Error writing to the socket!" << endl;
    }

}

string SocketCommunication::receive() {

    // Initialize buffer for the message
    char buffer[512];
    bzero(buffer, 512);

    // Receive message
    int status = read(socket_descriptor, buffer, 511);

    // Check if the action was successful
    if (status < 0) {
        cerr << "Error reading from the socket!" << endl;
    }

    // Return message
    return buffer;

}

void SocketCommunication::close_connection() {
    
    // Stop listening
    listening = false;

    // Join with listener
    listener.join();

    // Stop Ground Station remote control mode
    send("stop\n");

    // Stop the pass-throught OCU server
    send("\x07");
    
    // Close socket
    close(socket_descriptor);

}