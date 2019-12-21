#ifndef FOUR_WHEEL_ROBOT
#define FOUR_WHEEL_ROBOT

#include "Arduino.h"
#include "RobotBase.h"

//#define FOUR_WHEEL_ROBOT_DEBUG 1

class FourWheelRobot : public RobotBase {

  	public:
	  
	    FourWheelRobot(double wheel_separation_x,
						double wheel_separation_y,
						double wheel_radious);

		virtual void move(double velocity_x, double velocity_theta);
		virtual void stop();
        virtual void update(double dt);
					
	protected:

	    double wheel_separation_x_;
		double wheel_separation_y_;
	    double wheel_radious_;	

};
#endif