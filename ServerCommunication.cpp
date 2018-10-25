/* 
 * File:   ServerCommunication.cpp
 * Author: Jan Dufek
 */

#include "ServerCommunication.hpp"

ServerCommunication::ServerCommunication(FotokiteState * fotokiteState, const string send_pipe, const char * ip_address, const short port_receive) : Communication(fotokiteState) {

    // Initialize pipe for sending
    initializeSendPipe(send_pipe);
    
    // Initialize socket for receiving
    initializeReceiveSocket(ip_address, port_receive);
    
    // Initialize Fotokite remote control mode
    this->startRemoteControl();
    
    // Start listener
    this->startListener();

}

ServerCommunication::ServerCommunication(const ServerCommunication& orig) : Communication(orig) {
}

ServerCommunication::~ServerCommunication() {
    
    close_connection();
    
}

void ServerCommunication::initializeSendPipe(const string pipePath) {
        
    // Initialize pipe
    sendPipe.open(pipePath);
    
    cout << "Send pipe " << pipePath << " initialized." << endl;
    
}

void ServerCommunication::initializeReceiveSocket(const char * ip_address, const short port) {
    
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
    
    // Notify socket to start sending data
    string message = "GET /flog HTTP/1.0\r\n\r\n";
    status = write(socket_descriptor_receive, message.data(), message.size());
    
    // Check if the action was successful
    if (status < 0) {
        cerr << "Error initializing receive socket!" << endl;
    }
    
    cout << "Receive socket " << ip_address << ":" << to_string(port) << " initialized." << endl;
    
}

/**
 * Send message to Fotokite.
 * 
 * @param message
 */
void ServerCommunication::send(string message) {

    // Send message
    sendPipe << message;
    sendPipe.flush();
    
}

/**
 * Receive message from Fotokite.
 * 
 * @return 
 */
string ServerCommunication::receive() {

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
 * Close connection with Fotokite. Do not exit remote control mode as server is still using it.
 */
void ServerCommunication::close_connection() {
    
    // Stop listening
    listening = false;

    // Join with listener
    listener.join();
    
    // Close send pipe
    sendPipe.close();
    
    // Close receive socket
    close(socket_descriptor_receive);

}