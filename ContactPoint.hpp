/* 
 * File:   ContactPoint.hpp
 * Author: Jan Dufek
 *
 * Created on February 1, 2018, 4:29 PM
 */

#ifndef CONTACTPOINT_HPP
#define CONTACTPOINT_HPP

class ContactPoint {
public:
    ContactPoint(double, double, double, double, double);
    virtual ~ContactPoint();
    
    double x;
    double y;
    double z;
    double segmentTetherLength;
    double totalTetherLength;
    
private:

};

#endif /* CONTACTPOINT_HPP */

