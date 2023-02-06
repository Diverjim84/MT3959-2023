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

class LimeLight {
 
 
 std::shared_ptr<nt::NetworkTable> m_limelight = nt::NetworkTableInstance::GetDefault().GetTable("limelight");


public:
  LimeLight();

  bool IsTargetVisable();

  frc::Pose2d GetRobotPose();

  void SendData(LoggingLevel verbose);

  units::inch_t GetReflectiveTargetRange(double targetHight);


};
