#include "FourWheelRobot.h"

FourWheelRobot::FourWheelRobot(double wheel_separation_x,
						                    double wheel_separation_y,
						                    double wheel_radious): 
															wheel_separation_x_(wheel_separation_x),
                              wheel_separation_y_(wheel_separation_y),
                              wheel_radious_(wheel_radious)
{
}

void FourWheelRobot::move(double velocity_x, double velocity_theta)
{
  target_velocity_x_ = velocity_x;
  target_velocity_theta_ = velocity_theta;
}

void FourWheelRobot::stop()
{
	wheels_[0]->stop();
	wheels_[1]->stop();
 	wheels_[2]->stop();
	wheels_[3]->stop();
}

void FourWheelRobot::update(double dt) //TODO REMOVE THIS PARAMETER
{  
  double velocity_1  = 0;
  double velocity_2  = 0;
  double velocity_3  = 0;
  double velocity_4  = 0;

  double dtt = millis() - previous_command_time_;
  previous_command_time_ = millis();

  if (dtt <= COMMAND_TIMEOUT)
  { 
    double target_velocity_wx = 
      target_velocity_x_ / ( wheel_radious_); //angular velocity rad/sec
    double target_velocity_wtheta = 
      ( target_velocity_theta_ * (wheel_separation_x_ / 2.0 + wheel_separation_y_ / 2.0) ) / ( 2.0 * wheel_radious_ );

    velocity_1  = target_velocity_wx - target_velocity_wtheta;
    velocity_2  = target_velocity_wx + target_velocity_wtheta;
    velocity_3  = target_velocity_wx - target_velocity_wtheta;
    velocity_4  = target_velocity_wx + target_velocity_wtheta;  
  }
  
  if (wheels_.size() == 4) 
  {
    if (wheels_[0] != 0)
      wheels_[0]->move(velocity_1);
    if (wheels_[1] != 0)      
      wheels_[1]->move(velocity_2);
    if (wheels_[2] != 0)
      wheels_[2]->move(velocity_3);
    if (wheels_[3] != 0)      
      wheels_[3]->move(velocity_4);
  }

	#ifdef FOUR_WHEEL_ROBOT_DEBUG
  Serial.print("FourWheelRobot::move:");
  Serial.print(" velocity1\t");
  Serial.print(velocity_1);
  Serial.print(" velocity2\t");
  Serial.print(velocity_2);
  Serial.print(" velocity3\t");
  Serial.print(velocity_3);
  Serial.print(" velocity4\t");
  Serial.print(velocity_4);
  Serial.print("\n");
	#endif
    
  if (wheels_.size() == 4) 
  {
    if (wheels_[0] != 0)
      velocity_1 =  wheels_[0]->getVelocity();  
    if (wheels_[1] != 0)
      velocity_2 =  wheels_[1]->getVelocity();
    if (wheels_[2] != 0)
      velocity_3 =  wheels_[2]->getVelocity();  
    if (wheels_[3] != 0)
      velocity_4 =  wheels_[3]->getVelocity();
  }

	#ifdef FOUR_WHEEL_ROBOT_DEBUG
	Serial.print("FourWheelRobot::update:");
  Serial.print(" velocity1\t");
  Serial.print(velocity_1);
  Serial.print(" velocity2\t");
  Serial.print(velocity_2);
  Serial.print(" velocity3\t");
  Serial.print(velocity_3);
  Serial.print(" velocity4\t");
  Serial.print(velocity_4);
  Serial.print("\n");
	#endif

  vx_     = (  wheel_radious_ * ( velocity_1 + velocity_2 + velocity_3 + velocity_4 )  )   / 4.;  
	vy_     = 0;
  vtheta_ =  ( ( -velocity_1 + velocity_2 - velocity_3 + velocity_4 ) / 4.0 ) /  
              ( (wheel_separation_x_ / 2) + (wheel_separation_y_ / 2) )  ;
              
	#ifdef FOUR_WHEEL_ROBOT_DEBUG
	Serial.print("FourWheelRobot::update:");
  Serial.print(" vx\t");
  Serial.print(vx_);
  Serial.print(" vtheta\t");
  Serial.print(vtheta_);
  Serial.print("\n");
	#endif
  
  theta_  += vtheta_ * dt;
	x_      +=  vx_ * cos(theta_) * dtt;
	y_      +=  vx_ * sin(theta_) * dtt;

	#ifdef FOUR_WHEEL_ROBOT_DEBUG
	Serial.print("FourWheelRobot::update:");
  Serial.print(" x\t");
  Serial.print(x_);
  Serial.print(" y\t");
  Serial.print(y_);
  Serial.print(" theta\t");
  Serial.print(theta_);
  Serial.print("\n");
	#endif
}