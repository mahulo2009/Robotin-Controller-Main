#include <RosConfigL298N.h>

RosConfigL298N::RosConfigL298N(String ns):ns_(ns)
{
}

void RosConfigL298N::read(ros::NodeHandle &nh)
{
    nh.getParam(("/"+ns_+"/gain").c_str(), &this->gain);
    nh.getParam(("/"+ns_+"/power_min").c_str(), &this->power_min);
    nh.getParam(("/"+ns_+"/power_max").c_str(), &this->power_max);

    nh.getParam(("/"+ns_+"/pin_power_left").c_str(), &this->wheel_config[0].pin_power); 
    nh.getParam(("/"+ns_+"/pin_direction_left_1").c_str(), &this->wheel_config[0].pin_direction_1);
    nh.getParam(("/"+ns_+"/pin_direction_left_2").c_str(), &this->wheel_config[0].pin_direction_2);
    nh.getParam(("/"+ns_+"/pin_power_right").c_str(), &this->wheel_config[1].pin_power); 
    nh.getParam(("/"+ns_+"/pin_direction_right_1").c_str(), &this->wheel_config[1].pin_direction_1);
    nh.getParam(("/"+ns_+"/pin_direction_right_2").c_str(), &this->wheel_config[1].pin_direction_2);
}
