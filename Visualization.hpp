/* 
 * File:   Visualization.hpp
 * Author: Jan Dufek
 *
 * Created on October 10, 2018, 8:24 PM
 */

#ifndef VISUALIZATION_HPP
#define VISUALIZATION_HPP

#include <stdio.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <iostream>
#include <netdb.h>
#include <arpa/inet.h>

class Visualization {
public:
    Visualization(const char *, const short);
    Visualization(const Visualization& orig);
    virtual ~Visualization();
    
    void send(string);
    string receive();
    void close_connection();
    
private:
    
    void initializeSocket(const char *, const short);
    
    int socket_descriptor;
    struct sockaddr_in socket_address;

};

#endif /* VISUALIZATION_HPP */

