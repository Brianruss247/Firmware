/**
 * @file controller_base.h
 *
 * Base class definition for autopilot controller in chapter 6 of UAVbook, see http://uavbook.byu.edu/doku.php
 *
 * @author Gary Ellingson <gary.ellingson@byu.edu>
 */

#ifndef CONTROLLER_BASE_H
#define CONTROLLER_BASE_H

class controller_base
{
public:
    controller_base();
    void spin();
protected:
    virtual void control() = 0;
//private:

};

#endif // CONTROLLER_BASE_H
