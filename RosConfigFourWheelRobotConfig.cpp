#include <RosConfigFourWheelRobotConfig.h>

RosConfigFourWheelRobotConfig::RosConfigFourWheelRobotConfig(String ns):ns_(ns)
{
}

void RosConfigFourWheelRobotConfig::read(ros::NodeHandle &nh)
{
    nh.getParam(("/"+ns_+"/robot_wheel_separation_x").c_str(), &this->robot_wheel_separation_x);
    nh.getParam(("/"+ns_+"/robot_wheel_separation_y").c_str(), &this->robot_wheel_separation_y);
    nh.getParam(("/"+ns_+"/robot_wheel_radious").c_str(), &this->robot_wheel_radious);  
}
