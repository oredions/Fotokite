/* 
 * File:   SerialCommunication.hpp
 * Author: Jan Dufek
 *
 * Created on May 2, 2017, 1:18 PM
 */

#ifndef SERIALCOMMUNICATION_HPP
#define SERIALCOMMUNICATION_HPP

#include "Communication.hpp"
#include "Serial.h"
#include <thread>

class SerialCommunication : public Communication {
public:
    SerialCommunication(FotokiteState *, const char *);
    SerialCommunication(const SerialCommunication& orig);
    virtual ~SerialCommunication();
    
    void send(string);
    string receive();
    void close_connection();
    
private:
    
    Serial serial;
    
};

#endif /* SERIALCOMMUNICATION_HPP */

