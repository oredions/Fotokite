/* 
 * File:   SocketCommunication.hpp
 * Author: Jan Dufek
 */

#ifndef SOCKETCOMMUNICATION_HPP
#define SOCKETCOMMUNICATION_HPP

#include <stdio.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <iostream>
#include <netdb.h>
#include <arpa/inet.h>
#include <thread>

#include "Communication.hpp"

using namespace std;

class SocketCommunication : public Communication {
public:
    
    SocketCommunication(FotokiteState *, const char *, const short, const short);
    SocketCommunication(const SocketCommunication& orig);
    virtual ~SocketCommunication();
    
    void send(string);
    string receive();
    void close_connection();
    
private:
     
    void initializeSendSocket(const char *, const short);
    void initializeReceiveSocket(const char *, const short);
            
    int socket_descriptor_send;
    struct sockaddr_in socket_address_send;
    
    int socket_descriptor_receive;
    struct sockaddr_in socket_address_receive;

};

#endif /* SOCKETCOMMUNICATION_HPP */

