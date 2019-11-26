#ifndef Ros_Config_L298N_H
#define Ros_Config_L298N_H

#include <RosConfigBase.h>

struct wheel_config_dual_t {
    int pin_power;    
    int pin_direction_1;
    int pin_direction_2;
    int pin_encoder_1;    
    int pin_encoder_2;
};

class RosConfigL298N : public RosConfigBase {

  	public:

            RosConfigL298N(String ns);

            virtual void read(ros::NodeHandle &nh);

            float   gain;
            int     power_min;  
            int     power_max;
            int     ticks_per_revolution;   //TODO ENCODER NOTHING TO DO WITH L298N

            wheel_config_dual_t wheel_config[4];
            
	protected:

  	private:   
      
        String ns_;            
};
#endif