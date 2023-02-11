#pragma once


#include <frc/trajectory/Trajectory.h>
#include <frc/trajectory/TrajectoryGenerator.h>

#include "Constants.h"


class WaypointPoses{
    public:
        
        frc::Pose2d Red1Right{  610.77_in - 14.25_in - constants::swerveConstants::WheelBaseLength/2.0, 
                                42.19_in - 22_in, 
                                frc::Rotation2d(180_deg)
                             };
        frc::Pose2d Red1Center{ 610.77_in - 14.25_in - constants::swerveConstants::WheelBaseLength/2.0, 
                                42.19_in, 
                                frc::Rotation2d(180_deg)
                              };
        frc::Pose2d Red1Left{ 610.77_in - 14.25_in - constants::swerveConstants::WheelBaseLength/2.0, 
                              42.19_in + 22_in, 
                              frc::Rotation2d(180_deg)
                            };

        frc::Pose2d Red2Right{  610.77_in - 14.25_in - constants::swerveConstants::WheelBaseLength/2.0, 
                                108.19_in - 22_in, 
                                frc::Rotation2d(180_deg)
                             };
        frc::Pose2d Red2Center{ 610.77_in - 14.25_in - constants::swerveConstants::WheelBaseLength/2.0, 
                                108.19_in, 
                                frc::Rotation2d(180_deg)
                              };
        frc::Pose2d Red2Left{ 610.77_in - 14.25_in - constants::swerveConstants::WheelBaseLength/2.0, 
                              108.19_in + 22_in, 
                              frc::Rotation2d(180_deg)
                            };
        
        frc::Pose2d Red3Right{  610.77_in - 14.25_in - constants::swerveConstants::WheelBaseLength/2.0, 
                                147.19_in - 22_in, 
                                frc::Rotation2d(180_deg)
                             };
        frc::Pose2d Red3Center{ 610.77_in - 14.25_in - constants::swerveConstants::WheelBaseLength/2.0, 
                                147.19_in, 
                                frc::Rotation2d(180_deg)
                              };
        frc::Pose2d Red3Left{ 610.77_in - 14.25_in - constants::swerveConstants::WheelBaseLength/2.0, 
                              147.19_in + 22_in, 
                              frc::Rotation2d(180_deg)
                            };
        


        frc::Pose2d Blue8Right{  40.45_in + 14.25_in + constants::swerveConstants::WheelBaseLength/2.0, 
                                42.19_in + 22_in, 
                                frc::Rotation2d(0_deg)
                             };
        frc::Pose2d Blue8Center{ 40.45_in + 14.25_in + constants::swerveConstants::WheelBaseLength/2.0, 
                                42.19_in, 
                                frc::Rotation2d(0_deg)
                              };
        frc::Pose2d Blue8Left{ 40.45_in + 14.25_in + constants::swerveConstants::WheelBaseLength/2.0, 
                              42.19_in - 22_in, 
                              frc::Rotation2d(0_deg)
                            };

        frc::Pose2d Blue7Right{  40.45_in + 14.25_in + constants::swerveConstants::WheelBaseLength/2.0, 
                                108.19_in + 22_in, 
                                frc::Rotation2d(0_deg)
                             };
        frc::Pose2d Blue7Center{ 40.45_in + 14.25_in + constants::swerveConstants::WheelBaseLength/2.0, 
                                108.19_in, 
                                frc::Rotation2d(0_deg)
                              };
        frc::Pose2d Blue7Left{ 40.45_in + 14.25_in + constants::swerveConstants::WheelBaseLength/2.0, 
                              108.19_in - 22_in, 
                              frc::Rotation2d(0_deg)
                            };
        
        frc::Pose2d Blue6Right{  40.45_in + 14.25_in + constants::swerveConstants::WheelBaseLength/2.0, 
                                147.19_in + 22_in, 
                                frc::Rotation2d(0_deg)
                             };
        frc::Pose2d Blue6Center{ 40.45_in + 14.25_in + constants::swerveConstants::WheelBaseLength/2.0, 
                                147.19_in, 
                                frc::Rotation2d(0_deg)
                              };
        frc::Pose2d Blue6Left{ 40.45_in + 14.25_in + constants::swerveConstants::WheelBaseLength/2.0, 
                              147.19_in - 22_in, 
                              frc::Rotation2d(0_deg)
                            };
        

        frc::Translation2d BlueCorridorNear{ 2.2_m, 4.7_m};
        frc::Translation2d BlueCorridorFar{ 5.6_m, 4.7_m};

        frc::Translation2d BluePiece1{ 40.45_in + 14.25_in + 224_in, 180.19_in};
        frc::Translation2d BluePiece2{ 40.45_in + 14.25_in + 224_in, 132.19_in};
        frc::Translation2d BluePiece3{ 40.45_in + 14.25_in + 224_in,  84.19_in};
        frc::Translation2d BluePiece4{ 40.45_in + 14.25_in + 224_in,  36.19_in};


                
};
