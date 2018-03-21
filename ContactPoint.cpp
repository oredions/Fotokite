/* 
 * File:   ContactPoint.cpp
 * Author: Jan Dufek
 * 
 * Created on February 1, 2018, 4:29 PM
 */

#include "ContactPoint.hpp"

/**
 * Initialize contact point
 * 
 * @param contactPointX
 * @param contactPointY
 * @param contactPointZ
 * @param contactPointSegmentTetherLength Tether length between last contact point and the current contact point
 * @param contactPointTotalTetherLength Total tether length from this contact point to the ground control station
 */
ContactPoint::ContactPoint(double contactPointX, double contactPointY, double contactPointZ, double contactPointSegmentTetherLength, double contactPointTotalTetherLength) {
    
    this->x = contactPointX;
    this->y = contactPointY;
    this->z = contactPointZ;
    this->segmentTetherLength = contactPointSegmentTetherLength;
    this->totalTetherLength = contactPointTotalTetherLength;
    
}

ContactPoint::~ContactPoint() {
}

