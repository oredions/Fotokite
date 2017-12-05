/* 
 * File:   Communication.hpp
 * Author: Jan Dufek
 *
 * Created on May 2, 2017, 1:05 PM
 */

#ifndef COMMUNICATION_HPP
#define COMMUNICATION_HPP

#include "FotokiteState.hpp"
#include <thread>

class Communication {
public:
    Communication(FotokiteState *);
    Communication(const Communication& orig);
    virtual ~Communication();
    
    virtual void send(string) = 0;
    virtual string receive() = 0;
    virtual void close_connection() = 0;
    
protected:
        
    FotokiteState * fotokiteState;
        
    void startRemoteControl();
    void startListener();
    void listen();
    
    thread listener;
    
    bool listening;

};

#endif /* COMMUNICATION_HPP */