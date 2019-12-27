#include <RosConfigL298N.h>

RosConfigL298N::RosConfigL298N(String ns):ns_(ns)
{
}

void RosConfigL298N::read(ros::NodeHandle &nh)
{
    nh.getParam(("/"+ns_+"/gain").c_str(), &this->gain);
    nh.getParam(("/"+ns_+"/offset").c_str(), &this->offset);
    nh.getParam(("/"+ns_+"/power_min").c_str(), &this->power_min);
    nh.getParam(("/"+ns_+"/power_max").c_str(), &this->power_max);
    nh.getParam(("/"+ns_+"/ticks_per_revolution").c_str(), &this->ticks_per_revolution);
    
    nh.getParam(("/"+ns_+"/pin_power_front_left").c_str(), &this->wheel_config[0].pin_power); 
    nh.getParam(("/"+ns_+"/pin_direction_front_left_1").c_str(), &this->wheel_config[0].pin_direction_1);
    nh.getParam(("/"+ns_+"/pin_direction_front_left_2").c_str(), &this->wheel_config[0].pin_direction_2);
    nh.getParam(("/"+ns_+"/pin_encoder_front_left_1").c_str(), &this->wheel_config[0].pin_encoder_1);
    nh.getParam(("/"+ns_+"/pin_encoder_front_left_2").c_str(), &this->wheel_config[0].pin_encoder_2);

    nh.getParam(("/"+ns_+"/pin_power_front_right").c_str(), &this->wheel_config[1].pin_power); 
    nh.getParam(("/"+ns_+"/pin_direction_front_right_1").c_str(), &this->wheel_config[1].pin_direction_1);
    nh.getParam(("/"+ns_+"/pin_direction_front_right_2").c_str(), &this->wheel_config[1].pin_direction_2);
    nh.getParam(("/"+ns_+"/pin_encoder_front_right_1").c_str(), &this->wheel_config[1].pin_encoder_1);
    nh.getParam(("/"+ns_+"/pin_encoder_front_right_2").c_str(), &this->wheel_config[1].pin_encoder_2);

    nh.getParam(("/"+ns_+"/pin_power_back_left").c_str(), &this->wheel_config[2].pin_power); 
    nh.getParam(("/"+ns_+"/pin_direction_back_left_1").c_str(), &this->wheel_config[2].pin_direction_1);
    nh.getParam(("/"+ns_+"/pin_direction_back_left_2").c_str(), &this->wheel_config[2].pin_direction_2);
    nh.getParam(("/"+ns_+"/pin_encoder_back_left_1").c_str(), &this->wheel_config[2].pin_encoder_1);
    nh.getParam(("/"+ns_+"/pin_encoder_back_left_2").c_str(), &this->wheel_config[2].pin_encoder_2);

    nh.getParam(("/"+ns_+"/pin_power_back_right").c_str(), &this->wheel_config[3].pin_power); 
    nh.getParam(("/"+ns_+"/pin_direction_back_right_1").c_str(), &this->wheel_config[3].pin_direction_1);
    nh.getParam(("/"+ns_+"/pin_direction_back_right_2").c_str(), &this->wheel_config[3].pin_direction_2);
    nh.getParam(("/"+ns_+"/pin_encoder_back_right_1").c_str(), &this->wheel_config[3].pin_encoder_1);
    nh.getParam(("/"+ns_+"/pin_encoder_back_right_2").c_str(), &this->wheel_config[3].pin_encoder_2);
}