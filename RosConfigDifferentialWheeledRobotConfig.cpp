#include <RosConfigDifferentialWheeledRobotConfig.h>

RosConfigDifferentialWheeledRobotConfig::RosConfigDifferentialWheeledRobotConfig(String ns):ns_(ns)
{
}

void RosConfigDifferentialWheeledRobotConfig::read(ros::NodeHandle &nh)
{
    nh.getParam(("/"+ns_+"/robot_wheel_separation").c_str(), &this->robot_wheel_separation);
    nh.getParam(("/"+ns_+"/robot_wheel_radious").c_str(), &this->robot_wheel_radious);  
}
