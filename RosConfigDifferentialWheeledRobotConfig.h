#ifndef Ros_Config_Differential_Wheeled_Robot_Config_H
#define Ros_Config_Differential_Wheeled_Robot_Config_H

#include <RosConfigBase.h>

struct wheel_config_bldc_t {
    int can_id;
    int invert;
};

class RosConfigDifferentialWheeledRobotConfig : public RosConfigBase {

  	public:

            RosConfigDifferentialWheeledRobotConfig(String ns);

            virtual void read(ros::NodeHandle &nh);
           
            float robot_wheel_separation;
            float robot_wheel_radious;

	protected:

  	private:   
      
        String ns_;            
};
#endif