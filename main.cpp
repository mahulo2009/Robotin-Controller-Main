#include <Arduino.h>

#include <ros.h>  //TODO CORRECT DEPENDENCIES
#include <Pid.h>
#include <WheelEncoder.h>
#include <Encoder.h>
#include <RobotBase.h>
#include <DifferentialWheeledRobot.h>
#include <L298NHardwareController.h>
#include <RosController.h>
#include <RosAdapterRobot.h>
#include <RosConfigDifferentialWheeledRobotConfig.h>
#include <RosConfigL298N.h>
#include <TaskScheduler.h>

RosController *           ros_controller = 0;

RobotBase *               robot = 0; 

L298NHardwareController * controller1;
L298NHardwareController * controller2;

WheelBase *               wheel1 = 0;
WheelBase *               wheel2 = 0;

Encoder *                 encoder_1;
Encoder *                 encoder_2;

void robot_ros_publish_callback();
void robot_kinematic_callback();
void robot_close_loop_callback();

Task robot_ros_publish_task(250, TASK_FOREVER, &robot_ros_publish_callback);
Task robot_kinematic_task(100, TASK_FOREVER, &robot_kinematic_callback);
Task robot_close_loop_task(20, TASK_FOREVER, &robot_close_loop_callback);

Scheduler runner;

void createTask() { 
  runner.addTask(robot_ros_publish_task);
  Serial.println("robot_ros_publish_task task created");

  runner.addTask(robot_close_loop_task);
  Serial.println("robot_close_loop_task task created");

  runner.addTask(robot_kinematic_task);
  Serial.println("robot_kinematic_task task created");
}

void initTask() {
  Serial.println("robot_kinematic_task task  enabled");
 	robot_ros_publish_task.enable();

  Serial.println("robot_close_loop_task task enabled");
  robot_close_loop_task.enable();

  Serial.println("robot_kinematic_task task enabled"); 	
  robot_kinematic_task.enable();
}

void setup() {

  Serial.begin(115200);

  RosAdapterRobot * ros_adapter_robot = new RosAdapterRobot();

  ros_controller = new RosController();
  ros_controller->addNode(ros_adapter_robot);
  ros_controller->init();
 
  RosConfigDifferentialWheeledRobotConfig * ros_config_robot = new RosConfigDifferentialWheeledRobotConfig("robotin");
  ros_controller->readConfiguration(ros_config_robot);

  robot = 
        new DifferentialWheeledRobot(ros_config_robot->robot_wheel_separation,
                                      ros_config_robot->robot_wheel_radious);
    
  encoder_1 = new Encoder(22,23); //TODO READ ENCODER PIN FROM FILE

  RosConfigL298N * ros_config_motor = new RosConfigL298N("robotin");
  ros_controller->readConfiguration(ros_config_motor);

  controller1 = 
    new L298NHardwareController(ros_config_motor->gain,ros_config_motor->power_min,ros_config_motor->power_max);

  controller1->attachPower(ros_config_motor->wheel_config[0].pin_power);
  controller1->attachDirection(ros_config_motor->wheel_config[0].pin_direction_1,
                                ros_config_motor->wheel_config[0].pin_direction_2);
  controller1->attachEncoder(encoder_1);
 
  wheel1 = new WheelEncoder();
  wheel1->attachController(controller1);

  robot->addWheel(wheel1); 

  encoder_2 = new Encoder(2,3);

  controller2 = 
    new L298NHardwareController(ros_config_motor->gain,ros_config_motor->power_min,ros_config_motor->power_max);
   
  controller2->attachPower(ros_config_motor->wheel_config[1].pin_power);
  controller2->attachDirection(ros_config_motor->wheel_config[1].pin_direction_1,
                                ros_config_motor->wheel_config[1].pin_direction_2);
  controller2->attachEncoder(encoder_2);

  wheel2 = new WheelEncoder();
  wheel2->attachController(controller2);

  robot->addWheel(wheel2); 

  ros_adapter_robot->attachRobot(robot);

  createTask();
  initTask(); 
}

void robot_ros_publish_callback() 
{
  if (ros_controller != 0)
    ros_controller->update();
}

void robot_kinematic_callback() 
{
  if (wheel1 !=0)
    wheel1->update(0.1);
  if (wheel2 !=0)
    wheel2->update(0.1);
  if (robot != 0) 
    robot->update(0.1);
}

void robot_close_loop_callback()
{
  if (controller1 != 0) 
    controller1->update(0.02);
  if (controller2 != 0) 
    controller2->update(0.02);
}
  
void loop() {
  runner.execute();
}


//TODO CRETE A CLASS DEVICE.