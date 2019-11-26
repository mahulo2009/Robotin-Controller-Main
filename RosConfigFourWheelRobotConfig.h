#ifndef Ros_Config_FourWheelRobot_Config_H
#define Ros_Config_FourWheelRobot_Config_H

#include <RosConfigBase.h>

class RosConfigFourWheelRobotConfig : public RosConfigBase {

  	public:

            RosConfigFourWheelRobotConfig(String ns);

            virtual void read(ros::NodeHandle &nh);
           
            float robot_wheel_separation_x;
            float robot_wheel_separation_y;
            float robot_wheel_radious;

	protected:

  	private:   
      
        String ns_;            
};
#endif