// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Robot.h"

Robot::Robot():
  ll()
{

}

void Robot::RobotInit() {
  
  m_swerve.SetHeading(0_deg);
  headingControl =  true;
  driveMode = DriveMode::HeadingControl;

  m_swerve.ResetDriveEncoders();    
}

void Robot::UpdatePose(){

  m_swerve.UpdateOdometry();
  
  LL3DPose lpose = ll.GetRobotPose();
  if(lpose.validTarget){
    if((units::math::abs(m_swerve.GetChassisSpeeds().vx)+units::math::abs(m_swerve.GetChassisSpeeds().vy)) < .05_mps){
      //m_swerve.SetPose(lpose.botpose);
    } 
  }
}

void Robot::RobotPeriodic() {
  
  UpdatePose();
  ll.SendData(LoggingLevel::Basic);
  
  //m_swerve.SendData();

}

void Robot::AutonomousInit() {}
void Robot::AutonomousPeriodic() {}

void Robot::TeleopInit() {

  //Set current heading as target heading to avoid unwanted motion
  m_swerve.SetTargetHeading(m_swerve.GetHeading().Degrees());

}

/*
  Driver Back Button Resets Drive Pose
*/
void Robot::TeleopPeriodic() {
  
  //Reset Drive Pose
  if(driver.GetBackButtonPressed()){
    m_swerve.SetPose(frc::Pose2d(0_m, 0_m, frc::Rotation2d(0_deg)));
  }

  Drive();
}

void Robot::DisabledInit() {}
void Robot::DisabledPeriodic() {}

void Robot::TestInit() {}
void Robot::TestPeriodic() {}

void Robot::SimulationInit() {}
void Robot::SimulationPeriodic() {}


/*
Driver Control Function

Left Driver Joystick is heading/rotation
Right Driver Joystick is translation in x/y

Driver A Button is Heading Control, points in direction left joystick is pointing
Driver B Button is Velocity Control, just turns based on Left X axis

Driver Right Trigger Scales drive speed

*/
void Robot::Drive()
{
  double drivex = -driver.GetLeftY();
  double drivey = -driver.GetLeftX();
  //deadband
  if((fabs(drivex)+fabs(drivey))/2.0<.1){
    drivex = 0.0;
    drivey = 0.0;
  }

  //Get Right joystick for heading and velocity control
  double hy = -driver.GetRightX(); //hy is w for vel control
  double hx = -driver.GetRightY();

  //Scale Speed
  units::scalar_t scale = 1.0-driver.GetRightTriggerAxis()*.9;
  scale = scale*.8;
  
  //Set control mode
  if(driver.GetAButtonPressed()){
    driveMode = DriveMode::HeadingControl;
  }
  
  if(driver.GetBButtonPressed()){
    driveMode = DriveMode::VelocityMode;
  }

  switch(driveMode){
    case DriveMode::VelocityMode : 
      if(fabs(hy)<.1){//deadband rot vel, translations were done
        hy = 0.0;
      }
      m_swerve.Drive(
          m_xspeedLimiter.Calculate(pow(drivex,3)) * constants::swerveConstants::MaxSpeed * scale,
          m_yspeedLimiter.Calculate(pow(drivey,3)) * constants::swerveConstants::MaxSpeed * scale, 
          m_rotLimiter.Calculate(pow(hy,3)) * constants::swerveConstants::MaxAngularVelocity * scale, 
          true);
      break;
    case DriveMode::HeadingControl :
      if(sqrt(hx*hx+hy*hy) > 0.95)//make sure the joystick is begin used by calculating magnitude
      {
          m_swerve.SetTargetHeading(frc::Rotation2d(hx, hy).Degrees());
          frc::SmartDashboard::PutNumber("Heading Stick Value", sqrt(hx*hx+hy*hy));
      }
      m_swerve.DriveXY(
          m_xspeedLimiter.Calculate(pow(drivex,1)) * constants::swerveConstants::MaxSpeed * scale,
          m_yspeedLimiter.Calculate(pow(drivey,1)) * constants::swerveConstants::MaxSpeed * scale);
      break;

    default: break;
      
  }
}




#ifndef RUNNING_FRC_TESTS
int main() {
  return frc::StartRobot<Robot>();
}
#endif
