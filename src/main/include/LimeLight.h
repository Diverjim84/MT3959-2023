// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include "frc/smartdashboard/Smartdashboard.h"
#include "networktables/NetworkTable.h"
#include "networktables/NetworkTableInstance.h"
#include "networktables/NetworkTableEntry.h"
#include "networktables/NetworkTableValue.h"
#include <frc/geometry/Pose2d.h>
#include <units/length.h>
#include "LoggingLevel.h"


class LL3DPose{

public:
 LL3DPose(bool valT, frc::Pose2d p2d, std::vector<double> p3d): botpose{p2d}
 {
  validTarget = valT;
  pose3d = p3d;
 }

  bool validTarget;
  frc::Pose2d botpose;
  std::vector<double> pose3d;

};

class LimeLight {
 
 
 std::shared_ptr<nt::NetworkTable> m_limelight = nt::NetworkTableInstance::GetDefault().GetTable("limelight");


public:
  LimeLight();



  bool IsTargetVisable();

  LL3DPose GetRobotPose();

  void SendData(LoggingLevel verbose);

  units::inch_t GetReflectiveTargetRange(double targetHight);


};
