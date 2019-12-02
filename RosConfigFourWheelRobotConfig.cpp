#include <RosConfigFourWheelRobotConfig.h>

RosConfigFourWheelRobotConfig::RosConfigFourWheelRobotConfig(String ns):ns_(ns)
{
}

void RosConfigFourWheelRobotConfig::read(ros::NodeHandle &nh)
{
    nh.getParam(("/"+ns_+"/robot_wheel_separation_x").c_str(), &this->robot_wheel_separation_x);
    nh.getParam(("/"+ns_+"/robot_wheel_separation_y").c_str(), &this->robot_wheel_separation_y);
    nh.getParam(("/"+ns_+"/robot_wheel_radious").c_str(), &this->robot_wheel_radious);  
    nh.getParam(("/"+ns_+"/pid_p").c_str(), &this->pid_p);
    nh.getParam(("/"+ns_+"/pid_i").c_str(), &this->pid_i);
    nh.getParam(("/"+ns_+"/pid_d").c_str(), &this->pid_d);  
}
