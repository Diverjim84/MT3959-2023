// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Robot.h"


Robot::Robot()
{

}

void Robot::RobotInit() {
  
  m_autoChooser.SetDefaultOption(a_2PieceCorridor, AutoRoutine::k2PieceCorridor);
  m_autoChooser.AddOption(a_SimpleSwitch, AutoRoutine::kSimpleSwitch);
  m_autoChooser.AddOption(a_SpeedBump, AutoRoutine::kSpeedBump);
  m_autoChooser.AddOption(a_Test, AutoRoutine::kTest);
  frc::SmartDashboard::PutData("Auto Modes", &m_autoChooser);

  //m_swerve.SetHeading(0_deg);
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

  m_compressor.EnableAnalog(115.0_psi, 120.0_psi);

}

void Robot::UpdatePose(){

  m_swerve.UpdateOdometry();
  
}

bool Robot::FuseLL(){
  //If we are moving too fast, bail out
  if((units::math::abs(m_swerve.GetChassisSpeeds().vx)+units::math::abs(m_swerve.GetChassisSpeeds().vy)) > .02_mps){
    frc::SmartDashboard::PutString("Fuse LL", "To Fast");
    return false;
  } 
  

  //neither LL see target, exit
  if(!m_leftLL.IsTargetVisible() && !m_rightLL.IsTargetVisible()){
    frc::SmartDashboard::PutString("Fuse LL", "No Target");
    return false;
  }

  //Both LL see target, take the avg
  if(m_leftLL.IsTargetVisible() && m_rightLL.IsTargetVisible()){
    frc::SmartDashboard::PutString("Fuse LL", "Both");
    frc::Pose2d pos{(m_leftLL.GetRobotPose().X()+m_rightLL.GetRobotPose().X())/2.0,
                    (m_leftLL.GetRobotPose().Y()+m_rightLL.GetRobotPose().Y())/2.0,
                    frc::Rotation2d((m_leftLL.GetRobotPose().Rotation().Degrees()+m_rightLL.GetRobotPose().Rotation().Degrees())/2.0)
                    };
    m_swerve.SetPose(pos);
    return true;
  }
  //Just the left LL sees target
  if(m_leftLL.IsTargetVisible()){
    frc::SmartDashboard::PutString("Fuse LL", "Left Only");
    m_swerve.SetPose(m_leftLL.GetRobotPose());
    return true;
  }

  //Just the right LL sees target
  if(m_rightLL.IsTargetVisible()){
    frc::SmartDashboard::PutString("Fuse LL", "Right Only");
    m_swerve.SetPose(m_rightLL.GetRobotPose());
    return true;
  }
}


bool Robot::FuseLLNoHeading(){
  //If we are moving too fast, bail out
  if((units::math::abs(m_swerve.GetChassisSpeeds().vx)+units::math::abs(m_swerve.GetChassisSpeeds().vy)) > .05_mps){
    frc::SmartDashboard::PutString("Fuse LL", "To Fast");
    return false;
  } 
  
  frc::Rotation2d heading = m_swerve.GetHeading();

  //neither LL see target, exit
  if(!m_leftLL.IsTargetVisible() && !m_rightLL.IsTargetVisible()){
    frc::SmartDashboard::PutString("Fuse LL", "No Target");
    return false;
  }

  //Both LL see target, take the avg
  if(m_leftLL.IsTargetVisible() && m_rightLL.IsTargetVisible()){
    frc::SmartDashboard::PutString("Fuse LL", "Both");
    frc::Pose2d pos{(m_leftLL.GetRobotPose().X()+m_rightLL.GetRobotPose().X())/2.0,
                    (m_leftLL.GetRobotPose().Y()+m_rightLL.GetRobotPose().Y())/2.0,
                    heading
                    };
    m_swerve.SetPose(pos);
    return true;
  }


  //Just the left LL sees target
  if(m_leftLL.IsTargetVisible()){
    frc::SmartDashboard::PutString("Fuse LL", "Left Only");
    m_swerve.SetPose({m_leftLL.GetRobotPose().X(),m_leftLL.GetRobotPose().Y(), heading});
    return true;
  }

  //Just the right LL sees target
  if(m_rightLL.IsTargetVisible()){
    frc::SmartDashboard::PutString("Fuse LL", "Right Only");
    m_swerve.SetPose({m_rightLL.GetRobotPose().X(),m_rightLL.GetRobotPose().Y(), heading});
    return true;
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
  
  m_swerve.SendData();
  m_slide.SendData(LoggingLevel::Everything);
  m_arm.SendData(LoggingLevel::Everything);
  m_elevator.SendData(LoggingLevel::Everything);

  

}

void Robot::Gen2PieceCorridor(){

  //set Starting Pose
  if(!FuseLL())
  {
    if(frc::DriverStation::GetAlliance()== frc::DriverStation::Alliance::kBlue){
      m_swerve.SetPose({waypoints::BlueCorridorNear, frc::Rotation2d(180_deg)});
    }else{
      m_swerve.SetPose({waypoints::RedCorridorNear, frc::Rotation2d(0_deg)});
    }
  }
  frc::Pose2d x = m_swerve.GetPose();
  
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
                                                          frc::Pose2d(waypoints::BluePiece1.X(), waypoints::BluePiece1.Y()-12_in, frc::Rotation2d(0_deg)), 
                                                          c.config);
    
    //create waypoint list from piece pickup position to score position
    std::vector<frc::Translation2d> wp2Score2{  {waypoints::BlueCorridorFar.X(),waypoints::BlueCorridorFar.Y()-18_in}, 
                                                {waypoints::BlueCorridorNear.X(), waypoints::BlueCorridorNear.Y()-18_in}
                                                };

    //create traj from pickup position 1 to pickup score 2
    traj2Score2 = frc::TrajectoryGenerator::GenerateTrajectory(frc::Pose2d(waypoints::BluePiece1, frc::Rotation2d(180_deg)), 
                                                          wp2Score2, 
                                                          waypoints::Blue6Left, 
                                                          c.config);
  }else{//We are red
    //Intermediate waypoint from starting position to scoring position
    frc::Translation2d wp1 = ((waypoints::Red3Left - x).Translation())/2.0 + waypoints::Red3Left.Translation();

    //create required waypoints into a list
    std::vector<frc::Translation2d> wp2Score1{ wp1 };
    
    //create config for traj generation
    waypoints::WaypointPoses c{};

    //create traj from starting location to scoring position
    traj2Score1 = frc::TrajectoryGenerator::GenerateTrajectory(x, 
                                                          wp2Score1, 
                                                          waypoints::Red3Left, c.config);

    //create waypoint list from scoring position to piece pickup position
    std::vector<frc::Translation2d> wp2Piece1{ waypoints::RedCorridorNear, waypoints::RedCorridorFar };

    //create traj from score position 1 to pickup position 1
    traj2Piece1 = frc::TrajectoryGenerator::GenerateTrajectory(waypoints::Red3Left, 
                                                          wp2Piece1, 
                                                          frc::Pose2d(waypoints::RedPiece1, frc::Rotation2d(180_deg)), 
                                                          c.config);
    
    //create waypoint list from piece pickup position to score position
    std::vector<frc::Translation2d> wp2Score2{   {waypoints::RedCorridorFar.X(),waypoints::RedCorridorFar.Y()-18_in}, 
                                                {waypoints::RedCorridorNear.X(), waypoints::RedCorridorNear.Y()-18_in}
                                                 };

    //create traj from pickup position 1 to pickup score 2
    traj2Score2 = frc::TrajectoryGenerator::GenerateTrajectory(frc::Pose2d(waypoints::RedPiece1, frc::Rotation2d(180_deg)), 
                                                          wp2Score2, 
                                                          waypoints::Red3Right, 
                                                          c.config);
  }  

}

void Robot::Run2PieceCorridor(){
  RunSpeedBump();
  /*
  frc::Pose2d p; //goal pose
  //frc::Pose2d rp = m_swerve.GetPose(); //Current Pose
  units::degree_t heading; //Goal heading

  switch(autoState){
    case 0: autoState++; //Start timer for indexing into trajectory
            autoTimer.Reset(); //reset timer
            m_slide.SetPosition(0_in);
            //MidPos();
            //intensionally fall into next case statement
    case 1: p = traj2Score1.Sample(autoTimer.Get()).pose;
            if(frc::DriverStation::GetAlliance()==frc::DriverStation::Alliance::kBlue)
            {heading = 179.9_deg;}else{heading = 0_deg;} 
            m_swerve.DrivePos(p.X(), p.Y(), heading);
            if(autoTimer.Get()>(traj2Score1.TotalTime())){
                autoState++;
                autoTimer.Reset();
                m_swerve.DriveXY(0_mps, 0_mps);
                
            }
            break;
    case 2: //Place Piece
            m_swerve.DriveXY(0_mps, 0_mps);
            MidPos();
            if(autoTimer.Get()>.5_s){
              m_claw.ClawOpen();
            }
            if(autoTimer.Get()>1_s){
                autoState++;
                autoTimer.Reset();
            }
            break;
    case 3: autoState++; //Start timer for indexing into trajectory
            autoTimer.Reset(); //reset timer
            //intensionally fall into next case statement
    case 4: p = traj2Piece1.Sample(autoTimer.Get()).pose;
            if(autoTimer.Get()>1.5_s){
                GroundPos();
                m_claw.ClawClose();
                m_claw.SetIntakeSpeed(constants::clawConstants::FeedSpeed);
            }
            if(autoTimer.Get()>.5_s && autoTimer.Get()<1_s){
              heading = -90_deg; //Point toward charging station
            }
            if(autoTimer.Get()>2_s){
              if(frc::DriverStation::GetAlliance()==frc::DriverStation::Alliance::kBlue)
              {heading = -20_deg;}else{heading = -159.9_deg;} //Point toward next piece
            }
            m_swerve.DrivePos(p.X(), p.Y(), heading);
            if(autoTimer.Get()>(traj2Piece1.TotalTime())){
                autoState++;
                autoTimer.Reset();
            }
            break;
    case 5: //Pickup Piece
            m_swerve.DriveXY(0_mps, 0_mps);
            
            if(autoTimer.Get()>.5_s){
                m_claw.SetIntakeSpeed(constants::clawConstants::HoldSpeed);
                UpTuckPos();
                autoState++;
                autoTimer.Reset();
            }
            break;
    case 6: autoState++; //Start timer for indexing into trajectory
            autoTimer.Reset(); //reset timer
            //intensionally fall into next case statement
    case 7: p = traj2Score2.Sample(autoTimer.Get()).pose;
            if(frc::DriverStation::GetAlliance()==frc::DriverStation::Alliance::kBlue)
            {heading = 179.9_deg;}else{heading = 0_deg;} 
            m_swerve.DrivePos(p.X(), p.Y(), heading);
            if(autoTimer.Get()>(traj2Score2.TotalTime())){
                autoState++;
                autoTimer.Reset();
                MidPos();
            }
            break;
    case 8: //Place Piece
            //update Pose while still
            if(autoTimer.Get()<.25_s){
              m_swerve.DriveXY(0_mps, 0_mps);
              FuseLL();        
            }else{
              //if distance to goal is <3" drop the piece
              p = traj2Score2.Sample(traj2Piece2.TotalTime()).pose;
              if( p.Translation().Distance(m_swerve.GetPose().Translation())<.07_m){
                m_claw.ClawOpen();
                autoState++;
              }else{
                if(frc::DriverStation::GetAlliance()==frc::DriverStation::Alliance::kBlue)
                {heading = 179.9_deg;}else{heading = 0_deg;} 
                m_swerve.DrivePos(p.X(), p.Y(), heading);
              }
            }
            break;
    
    default: m_swerve.DriveXY(0_mps, 0_mps);
  }

*/

}

void Robot::GenSimpleSwitch(){
  //set Starting Pose
  if(!FuseLL())
  {
    if(frc::DriverStation::GetAlliance()== frc::DriverStation::Alliance::kBlue){
      m_swerve.SetPose({waypoints::Blue7Center.Translation(), frc::Rotation2d(180_deg)});
    }else{
      m_swerve.SetPose({waypoints::RedCorridorNear, frc::Rotation2d(0_deg)});
    }
  }
  frc::Pose2d x = m_swerve.GetPose();
  

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
                                                          frc::Pose2d(waypoints::BluePiece3.X(), waypoints::BluePiece3.Y()-12_in, frc::Rotation2d(0_deg)), 
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

  //FuseLLNoHeading();

  switch(autoState){
    case 0: autoState++; //Start timer for indexing into trajectory
            autoTimer.Reset(); //reset timer
            m_slide.SetPosition(0_in);
            //intensionally fall into next case statement
    case 1: p = traj2Score1.Sample(autoTimer.Get()).pose;
            if(frc::DriverStation::GetAlliance()==frc::DriverStation::Alliance::kBlue)
            {heading = 179.9_deg;}else{heading = 0_deg;} 
            m_swerve.DrivePos(p.X(), p.Y(), heading);
            if(autoTimer.Get()>(traj2Score1.TotalTime())){
                autoState++;
                autoTimer.Reset();
                m_swerve.DriveXY(0_mps, 0_mps);
                MidPos();
            }
            break;
    case 2: //Place Piece
            m_swerve.DriveXY(0_mps, 0_mps);
            if(autoTimer.Get()<.3_s){
              MidPos();
            }
            if(autoTimer.Get()>1_s){
              m_claw.ClawOpen();
            }
            if(autoTimer.Get()>1.25_s){
              UpTuckPos();
                autoState++;
                autoTimer.Reset();
            }
            break;
    case 3: autoState++; //Start timer for indexing into trajectory
            autoTimer.Reset(); //reset timer
            //intensionally fall into next case statement
    case 4: p = traj2Piece1.Sample(autoTimer.Get()).pose;
            if(m_swerve.GetPose().X() > (waypoints::BlueTapeX+constants::swerveConstants::WheelBaseLength) && 
               m_swerve.GetPose().X() < (waypoints::RedTapeX-constants::swerveConstants::WheelBaseLength)){
                GroundPos();
                m_claw.ClawClose();
                m_claw.SetIntakeSpeed(constants::clawConstants::FeedSpeed);
            }
            if(autoTimer.Get()<.75_s){
              //do nothing till we back off a bit
                heading = m_swerve.GetHeading().Degrees();
            }else{
              //if within 50" of peice, get ready
              if(m_swerve.GetPose().Translation().Distance(traj2Piece1.Sample(traj2Piece1.TotalTime()).pose.Translation())<78_in){
                GroundPos();
                if(frc::DriverStation::GetAlliance()==frc::DriverStation::Alliance::kBlue)
                {heading = 0_deg;}else{heading = 179.9_deg;} 
                m_claw.SetIntakeSpeed(constants::clawConstants::FeedSpeed);
              }else{
                TuckPos();
                heading = m_swerve.GetHeading().Degrees();
              }
            }
            
            m_swerve.DrivePos(p.X(), p.Y(), heading);
            if(autoTimer.Get()>(traj2Piece1.TotalTime())){
                autoState++;
                autoTimer.Reset();
            }
            break;
    case 5: //Pickup Piece
            m_swerve.DriveXY(0_mps, 0_mps);
            
            if(autoTimer.Get()>.5_s){
                TuckPos();
                autoState++;
                autoTimer.Reset();
            }
            break;
    case 6: autoState++; //Start timer for indexing into trajectory
            autoTimer.Reset(); //reset timer
            //intensionally fall into next case statement
    case 7: p = traj2Score2.Sample(autoTimer.Get()).pose;
            if(frc::DriverStation::GetAlliance()==frc::DriverStation::Alliance::kBlue)
            {heading = 179.9_deg;}else{heading = 0_deg;} 
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

void Robot::GenSpeedBump(){
//set Starting Pose
  if(!FuseLL())
  {
    if(frc::DriverStation::GetAlliance()== frc::DriverStation::Alliance::kBlue){
      m_swerve.SetPose({waypoints::BlueCorridorNear, frc::Rotation2d(180_deg)});
    }else{
      m_swerve.SetPose({waypoints::RedCorridorNear, frc::Rotation2d(0_deg)});
    }
  }
  frc::Pose2d x = m_swerve.GetPose();
  
  if(frc::DriverStation::GetAlliance()== frc::DriverStation::Alliance::kBlue){

    //Intermediate waypoint from starting position to scoring position
    frc::Translation2d wp1 = ((waypoints::Blue8Left - x).Translation())/2.0 + waypoints::Blue8Left.Translation();

    //create required waypoints into a list
    std::vector<frc::Translation2d> wp2Score1{ wp1 };
    
    //create config for traj generation
    waypoints::WaypointPoses c{};

    //create traj from starting location to scoring position
    traj2Score1 = frc::TrajectoryGenerator::GenerateTrajectory(x, 
                                                          wp2Score1, 
                                                          waypoints::Blue8Left, c.config);

    //create waypoint list from scoring position to piece pickup position
    std::vector<frc::Translation2d> wp2Piece1{ waypoints::BlueBumpNear, waypoints::BlueBumpFar };

    //create traj from score position 1 to pickup position 1
    traj2Piece1 = frc::TrajectoryGenerator::GenerateTrajectory(waypoints::Blue8Left, 
                                                          wp2Piece1, 
                                                          frc::Pose2d(waypoints::BluePiece4.X(), waypoints::BluePiece4.Y()-12_in, frc::Rotation2d(0_deg)), 
                                                          c.config);
    
    //create waypoint list from piece pickup position to score position
    std::vector<frc::Translation2d> wp2Score2{  {waypoints::BlueBumpFar.X(),waypoints::BlueBumpFar.Y()-12_in}, 
                                                {waypoints::BlueBumpNear.X(), waypoints::BlueBumpNear.Y()-12_in}
                                                };

    //create traj from pickup position 1 to pickup score 2
    traj2Score2 = frc::TrajectoryGenerator::GenerateTrajectory(frc::Pose2d(waypoints::BluePiece4, frc::Rotation2d(180_deg)), 
                                                          wp2Score2, 
                                                          waypoints::Blue8Right, 
                                                          c.config);
  }else{//We are red
    //Intermediate waypoint from starting position to scoring position
    frc::Translation2d wp1 = ((waypoints::Red1Left - x).Translation())/2.0 + waypoints::Red1Left.Translation();

    //create required waypoints into a list
    std::vector<frc::Translation2d> wp2Score1{ wp1 };
    
    //create config for traj generation
    waypoints::WaypointPoses c{};

    //create traj from starting location to scoring position
    traj2Score1 = frc::TrajectoryGenerator::GenerateTrajectory(x, 
                                                          wp2Score1, 
                                                          waypoints::Red1Left, c.config);

    //create waypoint list from scoring position to piece pickup position
    std::vector<frc::Translation2d> wp2Piece1{ waypoints::RedBumpNear, waypoints::RedBumpFar };

    //create traj from score position 1 to pickup position 1
    traj2Piece1 = frc::TrajectoryGenerator::GenerateTrajectory(waypoints::Red1Left, 
                                                          wp2Piece1, 
                                                          frc::Pose2d(waypoints::RedPiece4, frc::Rotation2d(180_deg)), 
                                                          c.config);
    
    //create waypoint list from piece pickup position to score position
    std::vector<frc::Translation2d> wp2Score2{   {waypoints::RedBumpFar.X(),waypoints::RedBumpFar.Y()-18_in}, 
                                                {waypoints::RedBumpNear.X(), waypoints::RedBumpNear.Y()-18_in}
                                                 };

    //create traj from pickup position 1 to pickup score 2
    traj2Score2 = frc::TrajectoryGenerator::GenerateTrajectory(frc::Pose2d(waypoints::RedPiece4, frc::Rotation2d(180_deg)), 
                                                          wp2Score2, 
                                                          waypoints::Red1Right, 
                                                          c.config);
  }  

}

void Robot::RunSpeedBump(){
  frc::Pose2d p; //goal pose
  //frc::Pose2d rp = m_swerve.GetPose(); //Current Pose
  units::degree_t heading; //Goal heading

  //FuseLLNoHeading();

  switch(autoState){
    case 0: autoState++; //Start timer for indexing into trajectory
            autoTimer.Reset(); //reset timer
            m_slide.SetPosition(0_in);
            MidPos();
            //intensionally fall into next case statement
    case 1: if(autoTimer.Get()>3_s){
                autoState++;
                autoTimer.Reset();
            }
            break;
    case 2: //Place Piece
            m_swerve.DriveXY(0_mps, 0_mps);
            if(autoTimer.Get()>0_s){
              m_claw.ClawOpen();
            }
            if(autoTimer.Get()>0.5_s){
                UpTuckPos();
            }
            if(autoTimer.Get()>2_s){
                autoState++;
                autoTimer.Reset();
            }
            break;
    case 3: //autoState++; //Start timer for indexing into trajectory
            autoTimer.Reset(); //reset timer
            //intensionally fall into next case statement
    case 4: p = traj2Piece1.Sample(autoTimer.Get()).pose;
            if(m_swerve.GetPose().X() > waypoints::BlueBumpFar.X() && m_swerve.GetPose().X() < waypoints::RedBumpFar.X()){
                GroundPos();
                m_claw.ClawClose();
                m_claw.SetIntakeSpeed(constants::clawConstants::FeedSpeed);
            }
            if(m_swerve.GetPose().X() > waypoints::BlueSwitchFar.X() && m_swerve.GetPose().X() < waypoints::RedSwitchFar.X()){
              if(frc::DriverStation::GetAlliance()==frc::DriverStation::Alliance::kBlue)
              {heading = -20_deg;}else{heading = -159.9_deg;} //Point toward next piece
            }
            m_swerve.DrivePos(p.X(), p.Y(), heading);
            if(autoTimer.Get()>(traj2Piece1.TotalTime())){
                autoState++;
                autoTimer.Reset();
            }
            break;
    case 5: //Pickup Piece
            m_swerve.DriveXY(0_mps, 0_mps);
            
            if(autoTimer.Get()>.5_s){
                m_claw.SetIntakeSpeed(constants::clawConstants::HoldSpeed);
                UpTuckPos();
                //autoState++;
                autoTimer.Reset();
            }
            break;
    case 6: autoState++; //Start timer for indexing into trajectory
            autoTimer.Reset(); //reset timer
            //intensionally fall into next case statement
    case 7: p = traj2Score2.Sample(autoTimer.Get()).pose;
            
            if(autoTimer.Get()>.5_s){
              if(frc::DriverStation::GetAlliance()==frc::DriverStation::Alliance::kBlue)
              {heading = 179.9_deg;}else{heading = 0_deg;} //Point toward goal
            }
             
            m_swerve.DrivePos(p.X(), p.Y(), heading);
            if(autoTimer.Get()>(traj2Score2.TotalTime())){
                autoState++;
                autoTimer.Reset();
                MidPos();
            }
            break;
    case 8: //Place Piece
            //update Pose while still
            if(autoTimer.Get()<.25_s){
              m_swerve.DriveXY(0_mps, 0_mps);
              FuseLL();        
            }else{
              //if distance to goal is <3" drop the piece
              p = traj2Score2.Sample(traj2Piece2.TotalTime()).pose;
              if( p.Translation().Distance(m_swerve.GetPose().Translation())<.07_m){
                m_claw.ClawOpen();
                autoState++;
              }else{
                if(frc::DriverStation::GetAlliance()==frc::DriverStation::Alliance::kBlue)
                {heading = 179.9_deg;}else{heading = 0_deg;} 
                //m_swerve.DrivePos(p.X(), p.Y(), heading);
              }
            }
            break;
    
    default: m_swerve.DriveXY(0_mps, 0_mps);
  }


}

void Robot::GenTest(){
  if(!FuseLL())
  {
    if(frc::DriverStation::GetAlliance()== frc::DriverStation::Alliance::kBlue){
      m_swerve.SetPose({waypoints::Blue7Center.Translation(), frc::Rotation2d(180_deg)});
    }else{
      m_swerve.SetPose({waypoints::RedCorridorNear, frc::Rotation2d(0_deg)});
    }
  }
  frc::Pose2d x = m_swerve.GetPose();
  
  
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
                                                          frc::Pose2d(waypoints::BlueSwitch.X()-12_in,waypoints::BlueSwitch.Y(), 180_deg), 
                                                          c.config);


}

void Robot::RunTest(){
  frc::Pose2d p; //goal pose
  //frc::Pose2d rp = m_swerve.GetPose(); //Current Pose
  units::degree_t heading; //Goal heading

  //FuseLLNoHeading();

  switch(autoState){
    case 0: autoState++; //Start timer for indexing into trajectory
            autoTimer.Reset(); //reset timer
            m_slide.SetPosition(0_in);
            //intensionally fall into next case statement
    case 1: p = traj2Score1.Sample(autoTimer.Get()).pose;
            if(frc::DriverStation::GetAlliance()==frc::DriverStation::Alliance::kBlue)
            {heading = 179.9_deg;}else{heading = 0_deg;} 
            m_swerve.DrivePos(p.X(), p.Y(), heading);
            if(autoTimer.Get()>(traj2Score1.TotalTime())){
                autoState++;
                autoTimer.Reset();
                m_swerve.DriveXY(0_mps, 0_mps);
                //MidPos();
            }
            break;
    case 2: //Place Piece
            m_swerve.DriveXY(0_mps, 0_mps);
            if(autoTimer.Get()<.3_s){
              MidPos();
            }
            if(autoTimer.Get()>1_s){
              m_claw.ClawOpen();
            }
            if(autoTimer.Get()>1.5_s){
              UpTuckPos();
            
                autoState++;
                autoTimer.Reset();
            }
            break;
    case 3: autoState++; //Start timer for indexing into trajectory
            autoTimer.Reset(); //reset timer
            break;
    case 4: p = traj2Piece1.Sample(autoTimer.Get()).pose;
            if(m_swerve.GetPose().X() > (waypoints::BlueTapeX+constants::swerveConstants::WheelBaseLength) && 
               m_swerve.GetPose().X() < (waypoints::RedTapeX-constants::swerveConstants::WheelBaseLength)){
                GroundPos();
                m_claw.ClawClose();
                m_claw.SetIntakeSpeed(constants::clawConstants::FeedSpeed);
            }
            if(autoTimer.Get()<.75_s){
              //do nothing till we back off a bit
                heading = m_swerve.GetHeading().Degrees();
            }else{
              //if within 50" of peice, get ready
              if(m_swerve.GetPose().Translation().Distance(traj2Piece1.Sample(traj2Piece1.TotalTime()).pose.Translation())<78_in){
                GroundPos();
                if(frc::DriverStation::GetAlliance()==frc::DriverStation::Alliance::kBlue)
                {heading = 0_deg;}else{heading = 179.9_deg;} 
                m_claw.SetIntakeSpeed(constants::clawConstants::FeedSpeed);
              }else{
                TuckPos();
                heading = m_swerve.GetHeading().Degrees();
              }
            }
            
            m_swerve.DrivePos(p.X(), p.Y(), heading);
            if(autoTimer.Get()>(traj2Piece1.TotalTime())){
                autoState++;
                autoTimer.Reset();
            }
            break;
    case 5: //Pickup Piece
            m_swerve.DriveXY(0_mps, 0_mps);
            
            if(autoTimer.Get()>.5_s){
                TuckPos();
                autoState++;
                autoTimer.Reset();
            }
            break;
    case 6: autoState++; //Start timer for indexing into trajectory
            autoTimer.Reset(); //reset timer
            //intensionally fall into next case statement
    case 7: p = traj2Score2.Sample(autoTimer.Get()).pose;
            if(frc::DriverStation::GetAlliance()==frc::DriverStation::Alliance::kBlue)
            {heading = 179.9_deg;}else{heading = 0_deg;} 
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
      case AutoRoutine::kSpeedBump : GenSpeedBump(); break;
      case AutoRoutine::kTest : GenTest(); break;
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
  std::this_thread::sleep_for(std::chrono::milliseconds(500));
  m_arm.SetAngle(50_deg);
  m_slide.SetPosition(0_in);
  m_elevator.SetHeight(1_in);

}

void Robot::GroundPos(){
  std::this_thread::sleep_for(std::chrono::milliseconds(500)); 
  m_arm.SetAngle(-90_deg);
  m_slide.SetPosition(0_in);
  m_elevator.SetHeight(0_in);

}

void Robot::MidPos(){
  std::this_thread::sleep_for(std::chrono::milliseconds(500)); 
  m_arm.SetAngle(40_deg);
  m_slide.SetPosition(0_in);
  m_elevator.SetHeight(1.5_in);
}

void Robot::HighPos(){
  std::this_thread::sleep_for(std::chrono::milliseconds(500)); 
  m_arm.SetAngle(25_deg);
  m_slide.SetPosition(14_in);
  m_elevator.SetHeight(19_in);
}

void Robot::TuckPos(){
  std::this_thread::sleep_for(std::chrono::milliseconds(500)); 
  m_arm.SetAngle(-130_deg);
  m_slide.SetPosition(0_in);
  m_elevator.SetHeight(4_in);

}

void Robot::UpTuckPos(){
  std::this_thread::sleep_for(std::chrono::milliseconds(500)); 
  m_arm.SetAngle(130_deg);
  m_slide.SetPosition(0_in);
  m_elevator.SetHeight(0_in);

}

void Robot::AutonomousInit() {
  autoTimer.Reset();
  autoTimer.Start();
  autoState = 0; 
  m_slide.SetPosition(0_in);
  m_claw.ClawClose();
  m_swerve.SetTargetHeading(m_swerve.GetHeading().Degrees());
  units::degree_t heading;
  if(frc::DriverStation::GetAlliance()==frc::DriverStation::Alliance::kBlue)
    {heading = 179.9_deg;}else{heading = 0_deg;}
  m_swerve.SetHeading(heading);
  m_swerve.SetTargetHeading(heading);
  GenTraj();

}

void Robot::AutonomousPeriodic() {
  
  switch(m_autoSelected){
      case AutoRoutine::k2PieceCorridor : Run2PieceCorridor();; break;
      case AutoRoutine::kSimpleSwitch : RunSimpleSwitch(); break;
      case AutoRoutine::kSpeedBump : RunSpeedBump(); break;
      case AutoRoutine::kTest : RunTest(); break;
      
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
  
  //m_swerve.SendData();
  if(m_slide.GetPosition() > 6_in || m_slide.GetTargetPos() > 6_in){
    m_swerve.SetRotationGain(.4);
  }else{
    m_swerve.SetRotationGain(.8);
  }

  //Reset Drive Pose
  if(driver.GetBackButtonPressed()){
    m_swerve.SetPose(frc::Pose2d(0_m, 0_m, frc::Rotation2d(0_deg)));
  }


  
  if(codriver.GetAButton()){
    m_claw.ClawOpen();
  }/*else{
    m_claw.ClawClose();
  }
*/
  if(codriver.GetBButton()){
    m_claw.ClawClose();
  }

  if(codriver.GetRightTriggerAxis()>0.0){
    m_claw.SetIntakeSpeed(codriver.GetRightTriggerAxis()*.2);//shoot
  }else{
    if(codriver.GetLeftTriggerAxis()>0.0){
      m_claw.SetIntakeSpeed(-codriver.GetLeftTriggerAxis()*.25);//feed
    }else{
      m_claw.SetIntakeSpeed(constants::clawConstants::HoldSpeed);
    }
  }

  
  //zero elevator
  if(codriver.GetStartButtonPressed()){
    m_elevator.SetSpeed(-.15);
    m_slide.SetSpeed(-.15);
  }

  if(codriver.GetPOV()>-1){
    switch(codriver.GetPOV()){
      case 0: HighPos();break;
      case 90: UpTuckPos();break;
      case 180: TuckPos();break;
      case 270: MidPos();break;
      default: break;
    }
  }else{
    if(codriver.GetRightBumperPressed()){
        GroundPos();
    }else{
      if(codriver.GetBackButtonPressed()){
        PickupPos();
      }
    }
  }
  


  Drive();
}

void Robot::DisabledInit() {}
void Robot::DisabledPeriodic() {
  //FuseLL();
}

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
      if(sqrt(hx*hx+hy*hy) > 0.9)//make sure the joystick is begin used by calculating magnitude
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
