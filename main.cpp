#include <Arduino.h>

#include <ros.h>  //TODO CORRECT DEPENDENCIES
#include <Pid.h>
#include <ImuBase.h>
#include <Imu.h>
#include <RosAdapterPid.h>
#include <RosAdapterImu.h>
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

#define ROS_PUB_TASK_MS     0.05  //20hz
#define CLOSE_LOOP_TASK_MS  0.05  //20hz

RosController *           ros_controller = 0;
RobotBase *               robot = 0;
ImuBase *                 imu = 0; 

L298NHardwareController * controller1;
L298NHardwareController * controller2;
L298NHardwareController * controller3;
L298NHardwareController * controller4;

WheelEncoder *            wheel1 = 0;
WheelEncoder *            wheel2 = 0;
WheelEncoder *            wheel3 = 0;
WheelEncoder *            wheel4 = 0;

Pid *                     pid1 = 0;
Pid *                     pid2 = 0;
Pid *                     pid3 = 0;
Pid *                     pid4 = 0;

Encoder *                 encoder_1;
Encoder *                 encoder_2;
Encoder *                 encoder_3;
Encoder *                 encoder_4;

void robot_ros_publish_callback();
void robot_kinematic_callback();
void robot_close_loop_callback();

Task robot_ros_publish_task(ROS_PUB_TASK_MS*1000, TASK_FOREVER, &robot_ros_publish_callback);
Task robot_close_loop_task(CLOSE_LOOP_TASK_MS*1000, TASK_FOREVER, &robot_close_loop_callback);

Scheduler runner;

void createTask() { 
  runner.addTask(robot_ros_publish_task);
  Serial.println("robot_ros_publish_task task created");

  runner.addTask(robot_close_loop_task);
  Serial.println("robot_close_loop_task task created");

}

void initTask() {
  Serial.println("robot_kinematic_task task  enabled");
 	robot_ros_publish_task.enable();

  Serial.println("robot_close_loop_task task enabled");
  robot_close_loop_task.enable();  
}

void setup() {

  Serial.begin(115200);

  RosAdapterRobot * ros_adapter_robot = new RosAdapterRobot();

  RosAdapterImu * ros_adapter_imu = new RosAdapterImu();

  //RosAdapterPid *   ros_adapter_pid_1   = new RosAdapterPid("/wheel1/cmd_vel","/wheel1/cmd_pid","/wheel1/tel_vel");
  //RosAdapterPid *   ros_adapter_pid_2   = new RosAdapterPid("/wheel2/cmd_vel","/wheel2/cmd_pid","/wheel2/tel_vel");
  //RosAdapterPid *   ros_adapter_pid_3   = new RosAdapterPid("/wheel3/cmd_vel","/wheel3/cmd_pid","/wheel3/tel_vel");
  //RosAdapterPid *   ros_adapter_pid_4   = new RosAdapterPid("/wheel4/cmd_vel","/wheel4/cmd_pid","/wheel4/tel_vel");

  ros_controller = new RosController();
  ros_controller->addNode(ros_adapter_robot);
  ros_controller->addNode(ros_adapter_imu);
  //ros_controller->addNode(ros_adapter_pid_1);
  //ros_controller->addNode(ros_adapter_pid_2);
  //ros_controller->addNode(ros_adapter_pid_3);
  //ros_controller->addNode(ros_adapter_pid_4);
  ros_controller->init();
 
  RosConfigFourWheelRobotConfig * ros_config_robot = new RosConfigFourWheelRobotConfig("robotin");
  ros_controller->readConfiguration(ros_config_robot);

  RosConfigL298N * ros_config_motor = new RosConfigL298N("robotin");
  ros_controller->readConfiguration(ros_config_motor);

  robot = 
    new FourWheelRobot(ros_config_robot->robot_wheel_separation_x,
                          ros_config_robot->robot_wheel_separation_y,
                          ros_config_robot->robot_wheel_radious);

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
  pid1 = new Pid();
  wheel1 = new WheelEncoder();

  //TODO FROM CONFIG
  pid1->setKp(0.5);
  pid1->setKi(3.1);
  pid1->setKd(0.0);
  
  controller1->attachPower(ros_config_motor->wheel_config[0].pin_power);
  controller1->attachDirection(ros_config_motor->wheel_config[0].pin_direction_1,
                                ros_config_motor->wheel_config[0].pin_direction_2);
  controller1->attachEncoder(encoder_1);
  wheel1->attachController(controller1);
  wheel1->attachPid(pid1);
  robot->addWheel(wheel1);
  //ros_adapter_pid_1->attachWheel(wheel1);

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
  pid2 = new Pid();
  wheel2 = new WheelEncoder();                

  pid2->setKp(0.5);
  pid2->setKi(3.1);
  pid2->setKd(0.0);
   
  controller2->attachPower(ros_config_motor->wheel_config[1].pin_power);
  controller2->attachDirection(ros_config_motor->wheel_config[1].pin_direction_1,
                                ros_config_motor->wheel_config[1].pin_direction_2);                                
  controller2->attachEncoder(encoder_2);
  wheel2->attachController(controller2);
  wheel2->attachPid(pid2);
  robot->addWheel(wheel2);
  //ros_adapter_pid_2->attachWheel(wheel2); 
  
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
  pid3 = new Pid();
  wheel3 = new WheelEncoder();      

  pid3->setKp(0.5);
  pid3->setKi(3.1);
  pid3->setKd(0.0);

  controller3->attachPower(ros_config_motor->wheel_config[2].pin_power);
  controller3->attachDirection(ros_config_motor->wheel_config[2].pin_direction_1,
                                ros_config_motor->wheel_config[2].pin_direction_2);
  controller3->attachEncoder(encoder_3);  
  wheel3->attachController(controller3);
  wheel3->attachPid(pid3);
  robot->addWheel(wheel3);
  //ros_adapter_pid_3->attachWheel(wheel3); 

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
  pid4 = new Pid();
  wheel4 = new WheelEncoder();
   
  pid4->setKp(0.5);
  pid4->setKi(3.1);
  pid4->setKd(0.0);
 
  controller4->attachPower(ros_config_motor->wheel_config[3].pin_power);
  controller4->attachDirection(ros_config_motor->wheel_config[3].pin_direction_1,
                                ros_config_motor->wheel_config[3].pin_direction_2);
  controller4->attachEncoder(encoder_4);
  wheel4->attachController(controller4);
  wheel4->attachPid(pid4);
  robot->addWheel(wheel4);
  //ros_adapter_pid_4->attachWheel(wheel4); 


  //Imu------------------------------------------------------------------------
  imu = new Imu();
  ros_adapter_imu->attachImu(imu);

  ros_adapter_robot->attachRobot(robot);

  createTask();
  initTask(); 
}

void robot_ros_publish_callback() 
{
  if (ros_controller != 0)
    ros_controller->update();
}

void robot_close_loop_callback()
{
  if (controller1 != 0) 
    controller1->update(CLOSE_LOOP_TASK_MS);
  if (controller2 != 0) 
    controller2->update(CLOSE_LOOP_TASK_MS);
  if (controller3 != 0) 
    controller3->update(CLOSE_LOOP_TASK_MS);
  if (controller4 != 0) 
    controller4->update(CLOSE_LOOP_TASK_MS);

  if (wheel1 != 0) 
    wheel1->update(CLOSE_LOOP_TASK_MS);
  if (wheel2 != 0) 
    wheel2->update(CLOSE_LOOP_TASK_MS);
  if (wheel3 != 0) 
    wheel3->update(CLOSE_LOOP_TASK_MS);
  if (wheel4 != 0) 
    wheel4->update(CLOSE_LOOP_TASK_MS);

  if (robot != 0) 
    robot->update(CLOSE_LOOP_TASK_MS); 

}
  
void loop() {
  runner.execute();
}