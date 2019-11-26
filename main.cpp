#include <Arduino.h>

#include <ros.h>  //TODO CORRECT DEPENDENCIES
#include <Pid.h>
#include <WheelEncoder.h>
#include <Encoder.h>
#include <RobotBase.h>
#include <FourWheelRobot.h>
#include <L298NHardwareController.h>
#include <RosController.h>
#include <RosAdapterRobot.h>
#include <RosConfigFourWheelRobotConfig.h>
#include <RosConfigL298N.h>
#include <TaskScheduler.h>

#define ROS_PUB_TASK_MS 50
#define KINEMATIC_TASK_MS 50
#define CLOSE_LOOP_TASK_MS 20

RosController *           ros_controller = 0;

RobotBase *               robot = 0; 

L298NHardwareController * controller1;
L298NHardwareController * controller2;
L298NHardwareController * controller3;
L298NHardwareController * controller4;

WheelBase *               wheel1 = 0;
WheelBase *               wheel2 = 0;
WheelBase *               wheel3 = 0;
WheelBase *               wheel4 = 0;

Encoder *                 encoder_1;
Encoder *                 encoder_2;
Encoder *                 encoder_3;
Encoder *                 encoder_4;

void robot_ros_publish_callback();
void robot_kinematic_callback();
void robot_close_loop_callback();

Task robot_ros_publish_task(ROS_PUB_TASK_MS, TASK_FOREVER, &robot_ros_publish_callback);
Task robot_kinematic_task(KINEMATIC_TASK_MS, TASK_FOREVER, &robot_kinematic_callback);
Task robot_close_loop_task(CLOSE_LOOP_TASK_MS, TASK_FOREVER, &robot_close_loop_callback);

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

  RosAdapterRobot * ros_adapter_robot = new RosAdapterRobot(true);

  ros_controller = new RosController();
  ros_controller->addNode(ros_adapter_robot);
  ros_controller->init();
 
  RosConfigFourWheelRobotConfig * ros_config_robot = new RosConfigFourWheelRobotConfig("robotin");
  ros_controller->readConfiguration(ros_config_robot);

  robot = 
        new FourWheelRobot(ros_config_robot->robot_wheel_separation_x,
                                      ros_config_robot->robot_wheel_separation_y,
                                      ros_config_robot->robot_wheel_radious);

  RosConfigL298N * ros_config_motor = new RosConfigL298N("robotin");
  ros_controller->readConfiguration(ros_config_motor);

  //Wheel Front Left------------------------------------------------------------------------
  controller1 = 
    new L298NHardwareController(ros_config_motor->gain,
                                ros_config_motor->power_min,
                                ros_config_motor->power_max,
                                ros_config_motor->ticks_per_revolution,
                                ros_config_robot->robot_wheel_radious);
  encoder_1 = 
    new Encoder(ros_config_motor->wheel_config[0].pin_encoder_1,
                ros_config_motor->wheel_config[0].pin_encoder_2);
  wheel1 = new WheelEncoder();

  controller1->attachPower(ros_config_motor->wheel_config[0].pin_power);
  controller1->attachDirection(ros_config_motor->wheel_config[0].pin_direction_1,
                                ros_config_motor->wheel_config[0].pin_direction_2);
  controller1->attachEncoder(encoder_1);
  wheel1->attachController(controller1);
  robot->addWheel(wheel1); 

  //Wheel Front Right------------------------------------------------------------------------
  controller2 = 
    new L298NHardwareController(ros_config_motor->gain,
                                ros_config_motor->power_min,
                                ros_config_motor->power_max,
                                ros_config_motor->ticks_per_revolution,
                                ros_config_robot->robot_wheel_radious);

  encoder_2 = 
    new Encoder(ros_config_motor->wheel_config[1].pin_encoder_1,
                ros_config_motor->wheel_config[1].pin_encoder_2);
  wheel2 = new WheelEncoder();                
   
  controller2->attachPower(ros_config_motor->wheel_config[1].pin_power);
  controller2->attachDirection(ros_config_motor->wheel_config[1].pin_direction_1,
                                ros_config_motor->wheel_config[1].pin_direction_2);                                
  controller2->attachEncoder(encoder_2);
  wheel2->attachController(controller2);
  robot->addWheel(wheel2); 
  
  //Wheel Back Left------------------------------------------------------------------------  
  controller3 = 
    new L298NHardwareController(ros_config_motor->gain,
                                  ros_config_motor->power_min,
                                  ros_config_motor->power_max,
                                  ros_config_motor->ticks_per_revolution,
                                  ros_config_robot->robot_wheel_radious);
  encoder_3 = 
    new Encoder(ros_config_motor->wheel_config[2].pin_encoder_1,
                ros_config_motor->wheel_config[2].pin_encoder_2);
  wheel3 = new WheelEncoder();                

  controller3->attachPower(ros_config_motor->wheel_config[2].pin_power);
  controller3->attachDirection(ros_config_motor->wheel_config[2].pin_direction_1,
                                ros_config_motor->wheel_config[2].pin_direction_2);
  controller3->attachEncoder(encoder_3);  
  wheel3->attachController(controller3);
  robot->addWheel(wheel3); 

  //Wheel Back Right------------------------------------------------------------------------
  controller4 = 
    new L298NHardwareController(ros_config_motor->gain,
                                ros_config_motor->power_min,
                                ros_config_motor->power_max,
                                ros_config_motor->ticks_per_revolution,
                                ros_config_robot->robot_wheel_radious);
  encoder_4 = 
    new Encoder(ros_config_motor->wheel_config[3].pin_encoder_1,
                ros_config_motor->wheel_config[3].pin_encoder_2);
  wheel4 = new WheelEncoder();
   
  controller4->attachPower(ros_config_motor->wheel_config[3].pin_power);
  controller4->attachDirection(ros_config_motor->wheel_config[3].pin_direction_1,
                                ros_config_motor->wheel_config[3].pin_direction_2);
  controller4->attachEncoder(encoder_4);
  wheel4->attachController(controller4);
  robot->addWheel(wheel4); 

  //Robot------------------------------------------------------------------------
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
    wheel1->update(KINEMATIC_TASK_MS/1000.0);
  if (wheel2 !=0)
    wheel2->update(KINEMATIC_TASK_MS/1000.0);
  if (wheel3 !=0)
    wheel3->update(KINEMATIC_TASK_MS/1000.0);
  if (wheel4 !=0)
    wheel4->update(KINEMATIC_TASK_MS/1000.0);

  if (robot != 0) 
    robot->update(KINEMATIC_TASK_MS/1000.0);
}

void robot_close_loop_callback()
{
  if (controller1 != 0) 
    controller1->update(CLOSE_LOOP_TASK_MS/1000.0);
  if (controller2 != 0) 
    controller2->update(CLOSE_LOOP_TASK_MS/1000.0);
  if (controller3 != 0) 
    controller3->update(CLOSE_LOOP_TASK_MS/1000.0);
  if (controller4 != 0) 
    controller4->update(CLOSE_LOOP_TASK_MS/1000.0);
}
  
void loop() {
  runner.execute();
}