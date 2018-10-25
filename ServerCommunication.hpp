/* 
 * File:   ServerCommunication.hpp
 * Author: Jan Dufek
 */

#ifndef SERVERCOMMUNICATION_HPP
#define SERVERCOMMUNICATION_HPP

#include <stdio.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <iostream>
#include <fstream>
#include <netdb.h>
#include <arpa/inet.h>
#include <thread>

#include "Communication.hpp"

using namespace std;

class ServerCommunication : public Communication {
public:
    
    ServerCommunication(FotokiteState * fotokiteState, const string pipe_send, const char * ip_address, const short port_receive);
    ServerCommunication(const ServerCommunication& orig);
    virtual ~ServerCommunication();
    
    void send(string);
    string receive();
    void close_connection();
    
private:
     
    void initializeSendPipe(const string pipePath);
    void initializeReceiveSocket(const char *, const short);
    
    ofstream sendPipe;
    
    int socket_descriptor_receive;
    struct sockaddr_in socket_address_receive;

};

#endif /* SERVERCOMMUNICATION_HPP */

