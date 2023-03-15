// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Robot.h"


Robot::Robot()
{

}

void Robot::RobotInit() {
  
  m_autoChooser.SetDefaultOption(a_SimpleSwitch, AutoRoutine::kSimpleSwitch);
  m_autoChooser.AddOption(a_2PieceCorridor, AutoRoutine::k2PieceCorridor);
  m_autoChooser.AddOption(a_2PieceCorridorSwitch, AutoRoutine::k2PieceCorridorSwitch);
  frc::SmartDashboard::PutData("Auto Modes", &m_autoChooser);

  m_swerve.SetHeading(0_deg);
  headingControl =  true;
  driveMode = DriveMode::HeadingControl;

  takeSpeedBump = false;
  autoState = 0;
  autoTimer.Reset();

  m_swerve.ResetDriveEncoders();  

  m_arm.Init();
  m_slide.Init();
  m_elevator.Init();
  m_claw.ClawClose();
  m_claw.SetIntakeSpeed(constants::clawConstants::HoldSpeed);

  m_compressor.EnableAnalog(110.0_psi, 115.0_psi);

}

void Robot::UpdatePose(){

  m_swerve.UpdateOdometry();
  
  if(m_leftLL.IsTargetVisible()){
    if((units::math::abs(m_swerve.GetChassisSpeeds().vx)+units::math::abs(m_swerve.GetChassisSpeeds().vy)) < .15_mps){
      //m_swerve.SetPose(m_leftLL.GetRobotPose());
    } 
  }

  

}

void Robot::RobotPeriodic() {
  
  UpdatePose();
  m_leftLL.SendData("Left LL", LoggingLevel::Basic);
  m_rightLL.SendData("Right LL", LoggingLevel::Basic);
  
  frc::Pose2d bp = m_swerve.GetPose();
  frc::SmartDashboard::PutNumber("Bot Pose X", units::inch_t( bp.X()).value());
  frc::SmartDashboard::PutNumber("Bot Pose Y", units::inch_t( bp.Y()).value());
  frc::SmartDashboard::PutNumber("Bot Pose Heading", bp.Rotation().Degrees().value());

  frc::SmartDashboard::PutNumber("Compressor Pressure (psi)", m_compressor.GetPressure().value());

  m_slide.SendData(LoggingLevel::Everything);
  m_arm.SendData(LoggingLevel::Everything);
  m_elevator.SendData(LoggingLevel::Everything);

  //m_swerve.SendData();

}

void Robot::Gen2PieceCorridor(){

  //set Starting Pose
  frc::Pose2d x = m_leftLL.GetRobotPose();
  m_swerve.SetPose(x); //Set the robot to this pose 

  
  if(frc::DriverStation::GetAlliance()== frc::DriverStation::Alliance::kBlue){

    //Intermediate waypoint from starting position to scoring position
    frc::Translation2d wp1 = ((waypoints::Blue6Right - x).Translation())/2.0 + waypoints::Blue6Right.Translation();

    //create required waypoints into a list
    std::vector<frc::Translation2d> wp2Score1{ wp1 };
    
    //create config for traj generation
    waypoints::WaypointPoses c{};

    //create traj from starting location to scoring position
    traj2Score1 = frc::TrajectoryGenerator::GenerateTrajectory(x, 
                                                          wp2Score1, 
                                                          waypoints::Blue6Right, c.config);

    //create waypoint list from scoring position to piece pickup position
    std::vector<frc::Translation2d> wp2Piece1{ waypoints::BlueCorridorNear, waypoints::BlueCorridorFar };

    //create traj from score position 1 to pickup position 1
    traj2Piece1 = frc::TrajectoryGenerator::GenerateTrajectory(waypoints::Blue6Right, 
                                                          wp2Piece1, 
                                                          frc::Pose2d(waypoints::BluePiece1, frc::Rotation2d(0_deg)), 
                                                          c.config);
    
    //create waypoint list from piece pickup position to score position
    std::vector<frc::Translation2d> wp2Score2{ waypoints::BlueCorridorFar, waypoints::BlueCorridorNear };

    //create traj from pickup position 1 to pickup score 2
    traj2Score2 = frc::TrajectoryGenerator::GenerateTrajectory(frc::Pose2d(waypoints::BluePiece1, frc::Rotation2d(180_deg)), 
                                                          wp2Score2, 
                                                          waypoints::Blue6Left, 
                                                          c.config);
  }else{//We are red
    //Intermediate waypoint from starting position to scoring position
    frc::Translation2d wp1 = ((waypoints::Red3Left - x).Translation())/2.0 + waypoints::Red3Right.Translation();

    //create required waypoints into a list
    std::vector<frc::Translation2d> wp2Score1{ wp1 };
    
    //create config for traj generation
    waypoints::WaypointPoses c{};

    //create traj from starting location to scoring position
    traj2Score1 = frc::TrajectoryGenerator::GenerateTrajectory(x, 
                                                          wp2Score1, 
                                                          waypoints::Red3Right, c.config);

    //create waypoint list from scoring position to piece pickup position
    std::vector<frc::Translation2d> wp2Piece1{ waypoints::RedCorridorNear, waypoints::RedCorridorFar };

    //create traj from score position 1 to pickup position 1
    traj2Piece1 = frc::TrajectoryGenerator::GenerateTrajectory(waypoints::Red3Right, 
                                                          wp2Piece1, 
                                                          frc::Pose2d(waypoints::RedPiece1, frc::Rotation2d(180_deg)), 
                                                          c.config);
    
    //create waypoint list from piece pickup position to score position
    std::vector<frc::Translation2d> wp2Score2{ waypoints::RedCorridorFar, waypoints::RedCorridorNear };

    //create traj from pickup position 1 to pickup score 2
    traj2Score2 = frc::TrajectoryGenerator::GenerateTrajectory(frc::Pose2d(waypoints::RedPiece1, frc::Rotation2d(180_deg)), 
                                                          wp2Score2, 
                                                          waypoints::Red3Left, 
                                                          c.config);
  }  

}

void Robot::Run2PieceCorridor(){
  
  frc::Pose2d p; //goal pose
  //frc::Pose2d rp = m_swerve.GetPose(); //Current Pose
  units::degree_t heading; //Goal heading

  switch(autoState){
    case 0: autoState++; //Start timer for indexing into trajectory
            autoTimer.Reset(); //reset timer
            //intensionally fall into next case statement
    case 1: p = traj2Score1.Sample(autoTimer.Get()).pose;
            if(frc::DriverStation::GetAlliance()==frc::DriverStation::Alliance::kBlue)
            {heading = 180_deg;}else{heading = 0_deg;} 
            m_swerve.DrivePos(p.X(), p.Y(), heading);
            if(autoTimer.Get()>(traj2Score1.TotalTime())){
                autoState++;
                autoTimer.Reset();
            }
            break;
    case 2: //Place Piece
            m_swerve.DriveXY(0_mps, 0_mps);
            autoState++;
            break;
    case 3: autoState++; //Start timer for indexing into trajectory
            autoTimer.Reset(); //reset timer
            //intensionally fall into next case statement
    case 4: p = traj2Piece1.Sample(autoTimer.Get()).pose;
            if(frc::DriverStation::GetAlliance()==frc::DriverStation::Alliance::kBlue)
            {heading = 0_deg;}else{heading = 180_deg;} 
            m_swerve.DrivePos(p.X(), p.Y(), heading);
            if(autoTimer.Get()>(traj2Piece1.TotalTime())){
                autoState++;
                autoTimer.Reset();
            }
            break;
    case 5: //Pickup Piece
            m_swerve.DriveXY(0_mps, 0_mps);
            autoState++;
            break;
    case 6: autoState++; //Start timer for indexing into trajectory
            autoTimer.Reset(); //reset timer
            //intensionally fall into next case statement
    case 7: p = traj2Score2.Sample(autoTimer.Get()).pose;
            if(frc::DriverStation::GetAlliance()==frc::DriverStation::Alliance::kBlue)
            {heading = 180_deg;}else{heading = 0_deg;} 
            m_swerve.DrivePos(p.X(), p.Y(), heading);
            if(autoTimer.Get()>(traj2Score2.TotalTime())){
                autoState++;
                autoTimer.Reset();
            }
            break;
    case 8: //Place Piece
            m_swerve.DriveXY(0_mps, 0_mps);
            autoState++;
            break;
    
    default: m_swerve.DriveXY(0_mps, 0_mps);
  }



}

void Robot::GenSimpleSwitch(){
  //set Starting Pose
  frc::Pose2d x = m_leftLL.GetRobotPose();
  m_swerve.SetPose(x); //Set the robot to this pose 


  if(frc::DriverStation::GetAlliance()== frc::DriverStation::Alliance::kBlue){

    //Intermediate waypoint from starting position to scoring position
    frc::Translation2d wp1 = ((waypoints::Blue7Right - x).Translation())/2.0 + waypoints::Blue7Right.Translation();

    //create required waypoints into a list
    std::vector<frc::Translation2d> wp2Score1{ wp1 };
    
    //create config for traj generation
    waypoints::WaypointPoses c{};

    //create traj from starting location to scoring position
    traj2Score1 = frc::TrajectoryGenerator::GenerateTrajectory(x, 
                                                          wp2Score1, 
                                                          waypoints::Blue7Right, c.config);

    //create waypoint list from scoring position to piece pickup position
    std::vector<frc::Translation2d> wp2Piece1{ waypoints::BlueSwitchNear, waypoints::BlueSwitchFar };

    //create traj from score position 1 to pickup position 1
    traj2Piece1 = frc::TrajectoryGenerator::GenerateTrajectory(waypoints::Blue7Right, 
                                                          wp2Piece1, 
                                                          frc::Pose2d(waypoints::BluePiece3, frc::Rotation2d(0_deg)), 
                                                          c.config);
    
    //create waypoint list from piece pickup position to score position
    std::vector<frc::Translation2d> wp2Score2{ waypoints::BlueSwitchFar};

    //create traj from pickup position 1 to pickup score 2
    traj2Score2 = frc::TrajectoryGenerator::GenerateTrajectory(frc::Pose2d(waypoints::BluePiece3, frc::Rotation2d(180_deg)), 
                                                          wp2Score2, 
                                                          frc::Pose2d(waypoints::BlueSwitch, 180_deg), 
                                                          c.config);
  }else{//We are red
    //Intermediate waypoint from starting position to scoring position
    frc::Translation2d wp1 = ((waypoints::Red2Left - x).Translation())/2.0 + waypoints::Red2Right.Translation();

    //create required waypoints into a list
    std::vector<frc::Translation2d> wp2Score1{ wp1 };
    
    //create config for traj generation
    waypoints::WaypointPoses c{};

    //create traj from starting location to scoring position
    traj2Score1 = frc::TrajectoryGenerator::GenerateTrajectory(x, 
                                                          wp2Score1, 
                                                          waypoints::Red2Right, c.config);

    //create waypoint list from scoring position to piece pickup position
    std::vector<frc::Translation2d> wp2Piece1{ waypoints::RedSwitchNear, waypoints::RedSwitchFar };

    //create traj from score position 1 to pickup position 1
    traj2Piece1 = frc::TrajectoryGenerator::GenerateTrajectory(waypoints::Red2Right, 
                                                          wp2Piece1, 
                                                          frc::Pose2d(waypoints::RedPiece3, frc::Rotation2d(180_deg)), 
                                                          c.config);
    
    //create waypoint list from piece pickup position to score position
    std::vector<frc::Translation2d> wp2Score2{ waypoints::RedSwitchFar};

    //create traj from pickup position 1 to pickup score 2
    traj2Score2 = frc::TrajectoryGenerator::GenerateTrajectory(frc::Pose2d(waypoints::RedPiece3, frc::Rotation2d(180_deg)), 
                                                          wp2Score2, 
                                                          frc::Pose2d(waypoints::RedSwitch, 0_deg), 
                                                          c.config);
  }  


}

void Robot::RunSimpleSwitch(){
    frc::Pose2d p; //goal pose
  //frc::Pose2d rp = m_swerve.GetPose(); //Current Pose
  units::degree_t heading; //Goal heading

  switch(autoState){
    case 0: autoState++; //Start timer for indexing into trajectory
            autoTimer.Reset(); //reset timer
            //intensionally fall into next case statement
    case 1: p = traj2Score1.Sample(autoTimer.Get()).pose;
            if(frc::DriverStation::GetAlliance()==frc::DriverStation::Alliance::kBlue)
            {heading = 180_deg;}else{heading = 0_deg;} 
            m_swerve.DrivePos(p.X(), p.Y(), heading);
            if(autoTimer.Get()>(traj2Score1.TotalTime())){
                autoState++;
                autoTimer.Reset();
            }
            break;
    case 2: //Place Piece
            m_swerve.DriveXY(0_mps, 0_mps);
            autoState++;
            break;
    case 3: autoState++; //Start timer for indexing into trajectory
            autoTimer.Reset(); //reset timer
            //intensionally fall into next case statement
    case 4: p = traj2Piece1.Sample(autoTimer.Get()).pose;
            if(frc::DriverStation::GetAlliance()==frc::DriverStation::Alliance::kBlue)
            {heading = 0_deg;}else{heading = 180_deg;} 
            m_swerve.DrivePos(p.X(), p.Y(), heading);
            if(autoTimer.Get()>(traj2Piece1.TotalTime())){
                autoState++;
                autoTimer.Reset();
            }
            break;
    case 5: //Pickup Piece
            m_swerve.DriveXY(0_mps, 0_mps);
            autoState++;
            break;
    case 6: autoState++; //Start timer for indexing into trajectory
            autoTimer.Reset(); //reset timer
            //intensionally fall into next case statement
    case 7: p = traj2Score2.Sample(autoTimer.Get()).pose;
            if(frc::DriverStation::GetAlliance()==frc::DriverStation::Alliance::kBlue)
            {heading = 180_deg;}else{heading = 0_deg;} 
            m_swerve.DrivePos(p.X(), p.Y(), heading);
            if(autoTimer.Get()>(traj2Score2.TotalTime())){
                autoState++;
                autoTimer.Reset();
            }
            break;
    case 8: //Place Piece
            m_swerve.DriveXY(0_mps, 0_mps);
            autoState++;
            break;
    
    default: m_swerve.DriveXY(0_mps, 0_mps);
  }
}
void Robot::GenTraj(){
    m_autoSelected = m_autoChooser.GetSelected();
    
    switch(m_autoSelected){
      case AutoRoutine::k2PieceCorridor : Gen2PieceCorridor(); break;
      case AutoRoutine::kSimpleSwitch : GenSimpleSwitch(); break;
      
      
    }
}

void Robot::TrackToGoal(frc::Pose2d goal){
  if(m_leftLL.IsTargetVisible()){
    if((units::math::abs(m_swerve.GetChassisSpeeds().vx)+units::math::abs(m_swerve.GetChassisSpeeds().vy)) < .05_mps){
      m_swerve.SetPose(m_leftLL.GetRobotPose());
    }

    m_swerve.DrivePos(goal.X(), goal.Y(), goal.Rotation().Degrees());

  }


}

void Robot::PickupPos(){
  m_arm.SetAngle(60_deg);
  m_slide.SetPosition(0_in);
  m_elevator.SetHeight(0_in);

}

void Robot::GroundPos(){
  m_arm.SetAngle(-90_deg);
  m_slide.SetPosition(0_in);
  m_elevator.SetHeight(0_in);

}

void Robot::MidPos(){
  m_arm.SetAngle(40_deg);
  m_slide.SetPosition(0_in);
  m_elevator.SetHeight(0_in);
}

void Robot::HighPos(){
  m_arm.SetAngle(25_deg);
  m_slide.SetPosition(14_in);
  m_elevator.SetHeight(18_in);
}

void Robot::TuckPos(){
  m_arm.SetAngle(-130_deg);
  m_slide.SetPosition(0_in);
  m_elevator.SetHeight(0_in);

}

void Robot::UpTuckPos(){
  m_arm.SetAngle(130_deg);
  m_slide.SetPosition(0_in);
  m_elevator.SetHeight(0_in);

}
void Robot::AutonomousInit() {
  autoTimer.Reset();
  autoTimer.Start();
  autoState = 0; 
  
  Gen2PieceCorridor();

}
void Robot::AutonomousPeriodic() {
  
  switch(m_autoSelected){
      case AutoRoutine::k2PieceCorridor : Run2PieceCorridor();; break;
      case AutoRoutine::kSimpleSwitch : RunSimpleSwitch(); break;
      
  }

  frc::SmartDashboard::PutNumber("Auto State", autoState);
  frc::SmartDashboard::PutNumber("Auto Timer", autoTimer.Get().value());
  frc::Pose2d tp = traj2Score1.Sample(autoTimer.Get()).pose;
  frc::SmartDashboard::PutNumber("Target Pose X", units::inch_t( tp.X()).value());
  frc::SmartDashboard::PutNumber("Target Pose Y", units::inch_t( tp.Y()).value());
  frc::SmartDashboard::PutNumber("Target Pose Heading", tp.Rotation().Degrees().value());
}

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


  if(codriver.GetBButtonPressed()){
    m_claw.ClawClose();
  }else{
    if(codriver.GetXButtonPressed()){
      m_claw.ClawOpen();
    }
  }
  if(codriver.GetAButtonPressed()){
    m_claw.ClawToggle();
  }
  if(codriver.GetRightTriggerAxis()>0.0){
    m_claw.SetIntakeSpeed(codriver.GetRightTriggerAxis()*.3);
  }else{
    if(codriver.GetLeftTriggerAxis()>0.0){
      m_claw.SetIntakeSpeed(-codriver.GetLeftTriggerAxis()*.3);
    }else{
      m_claw.SetIntakeSpeed(constants::clawConstants::HoldSpeed);
    }
  }

  
  //zero elevator
  if(codriver.GetStartButtonPressed()){
    m_elevator.SetSpeed(-.15);
  }

  if(codriver.GetPOV()>-1){
    switch(codriver.GetPOV()){
      case 0: HighPos();break;
      case 90: UpTuckPos();break;
      case 180: GroundPos();break;
      case 270: MidPos();break;
      default: break;
    }
  }else{
    if(codriver.GetRightBumperPressed()){
      PickupPos();
    }else{
      if(codriver.GetBackButtonPressed()){
        TuckPos();
      }
    }
  }
  if(codriver.GetLeftBumper()){
    m_arm.SetSpeed(-codriver.GetLeftY());
    m_elevator.SetSpeed( -codriver.GetRightY()/4.0);
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
  scale = scale;
  
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
    case DriveMode::TargetTracking : 
      if(driver.GetAButton())
      {
        if(frc::DriverStation::GetAlliance()==frc::DriverStation::Alliance::kBlue){
          TrackToGoal(frc::Pose2d(waypoints::BlueSwitch, frc::Rotation2d(180_deg)));
        }else{
          TrackToGoal(frc::Pose2d(waypoints::RedSwitch, frc::Rotation2d(0_deg)));
        }
      }
      break;
    default: break;
      
  }
}




#ifndef RUNNING_FRC_TESTS
int main() {
  return frc::StartRobot<Robot>();
}
#endif
