/* 
 * File:   SocketCommunication.cpp
 * Author: Jan Dufek
 */

#include "SocketCommunication.hpp"

SocketCommunication::SocketCommunication(FotokiteState * fotokiteState, const char * ip_address, const short port_send, const short port_receive) : Communication(fotokiteState) {
     
    // Initialize socket for sending
    initializeSendSocket(ip_address, port_send);
    
    // Initialize socket for receiving
    initializeReceiveSocket(ip_address, port_receive);
    
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

void SocketCommunication::initializeSendSocket(const char * ip_address, const short port) {
    
    // Create socket descriptor. We want to use datagram UDP.
    socket_descriptor_send = socket(AF_INET, SOCK_STREAM, 0);

    if (socket_descriptor_send < 0) {
        cout << "Error creating send socket descriptor." << endl;
    }

    // Create socket address
    socket_address_send.sin_family = AF_INET;
    socket_address_send.sin_addr.s_addr = inet_addr(ip_address);
    socket_address_send.sin_port = htons(port);

    // Connect to the server
    int status = connect(socket_descriptor_send, (struct sockaddr *) &socket_address_send, sizeof (socket_address_send));

    // Check if the connection was established
    if (status < 0) {
        cerr << "Error connecting to the TCP server at IP address " << ip_address << " at port " << to_string(port) << "! Make sure your machine is connected to the OCU server via ethernet and has IP address on the same network." << endl;
    }
    
}

void SocketCommunication::initializeReceiveSocket(const char * ip_address, const short port) {
    
    // Create socket descriptor. We want to use datagram UDP.
    socket_descriptor_receive = socket(AF_INET, SOCK_STREAM, 0);

    if (socket_descriptor_receive < 0) {
        cout << "Error creating receive socket descriptor." << endl;
    }

    // Create socket address
    socket_address_receive.sin_family = AF_INET;
    socket_address_receive.sin_addr.s_addr = inet_addr(ip_address);
    socket_address_receive.sin_port = htons(port);

    // Connect to the server
    int status = connect(socket_descriptor_receive, (struct sockaddr *) &socket_address_receive, sizeof (socket_address_receive));

    // Check if the connection was established
    if (status < 0) {
        cerr << "Error connecting to the TCP server at IP address " << ip_address << " at port " << to_string(port) << "! Make sure your machine is connected to the OCU server via ethernet and has IP address on the same network." << endl;
    }
    
}

/**
 * Send message to Fotokite.
 * 
 * @param message
 */
void SocketCommunication::send(string message) {

    // Send message
    int status = write(socket_descriptor_send, message.data(), message.size());

    // Check if the action was successful
    if (status < 0) {
        cerr << "Error writing to the socket!" << endl;
    }

}

/**
 * Receive message from Fotokite.
 * 
 * @return 
 */
string SocketCommunication::receive() {

    // Initialize buffer for the message
    char buffer[512];
    bzero(buffer, 512);

    // Receive message
    int status = read(socket_descriptor_receive, buffer, 511);

    // Check if the action was successful
    if (status < 0) {
        cerr << "Error reading from the socket!" << endl;
    }

    // Return message
    return buffer;

}

/**
 * Close connection with Fotokite.
 */
void SocketCommunication::close_connection() {
    
    // Stop listening
    listening = false;

    // Join with listener
    listener.join();

    // Stop Ground Station remote control mode
    send("stop\n");

    // Stop the pass-throught OCU server
    send("\x07");
    
    // Close send socket
    close(socket_descriptor_send);
    
    // Close receive socket
    close(socket_descriptor_receive);

}