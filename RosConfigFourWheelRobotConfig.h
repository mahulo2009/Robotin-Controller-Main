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

            float pid_p;
            float pid_d;
            float pid_i;

	protected:

  	private:   
      
        String ns_;            
};
#endif