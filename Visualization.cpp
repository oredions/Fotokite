/* 
 * File:   Visualization.cpp
 * Author: Jan Dufek
 * 
 * Created on October 10, 2018, 8:24 PM
 */

#include "Visualization.hpp"

Visualization::Visualization(const char * ip_address, const short port_send) {
    
    initializeSocket(ip_address, port_send);
    
}

Visualization::Visualization(const Visualization& orig) {
}

Visualization::~Visualization() {
    
    close_connection();
    
}

void Visualization::initializeSocket(const char * ip_address, const short port) {
    
    // Create socket descriptor. We want to use datagram UDP.
    socket_descriptor = socket(AF_INET, SOCK_STREAM, 0);

    if (socket_descriptor < 0) {
        cout << "Error creating visualization socket descriptor." << endl;
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
    
}

/**
 * Send message to visualization server.
 * 
 * @param message
 */
void Visualization::send(string message) {

    // Send message
    int status = write(socket_descriptor, message.data(), message.size());

    // Check if the action was successful
    if (status < 0) {
        cerr << "Error writing to the socket!" << endl;
    }

}

/**
 * Receive message from visualization server.
 * 
 * @return 
 */
string Visualization::receive() {

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

/**
 * Close connection with visualization server.
 */
void Visualization::close_connection() {
    
    // Close send socket
    close(socket_descriptor);

}