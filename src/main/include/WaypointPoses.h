#pragma once


#include <frc/trajectory/Trajectory.h>
#include <frc/trajectory/TrajectoryGenerator.h>
#include <frc/trajectory/constraint/RectangularRegionConstraint.h>
#include <frc/trajectory/constraint/MaxVelocityConstraint.h>

#include <units/math.h>
#include <units/velocity.h>
#include <units/length.h>
#include <units/dimensionless.h>
#include <units/angular_velocity.h>
#include <units/acceleration.h>

#include "frc/geometry/Rotation2d.h"
#include "frc/geometry/Translation2d.h"
#include "frc/trajectory/constraint/TrajectoryConstraint.h"

#include "Constants.h"


class WaypointPoses{
  public:
    WaypointPoses(){
      //Bottom Left, Top Right, speed limit
      frc::RectangularRegionConstraint slowRegionRedSwitch{frc::Translation2d{452_in, 59_in},
                                                  frc::Translation2d{537_in, 155_in},
                                                  frc::MaxVelocityConstraint{.5_mps}};

      frc::RectangularRegionConstraint slowRegionRedBump{frc::Translation2d{491_in, 0_in},
                                                  frc::Translation2d{505_in, 59_in},
                                                  frc::MaxVelocityConstraint{.25_mps}};

      frc::RectangularRegionConstraint slowRegionBlueSwitch{frc::Translation2d{113_in, 59_in},
                                                  frc::Translation2d{200_in, 155_in},
                                                  frc::MaxVelocityConstraint{.5_mps}};

      frc::RectangularRegionConstraint slowRegionBlueBump{frc::Translation2d{145_in, 0_in},
                                                  frc::Translation2d{155_in, 59_in},
                                                  frc::MaxVelocityConstraint{.25_mps}};
      
      config.AddConstraint(slowRegionRedSwitch);
      config.AddConstraint(slowRegionRedBump);
      config.AddConstraint(slowRegionBlueSwitch);
      config.AddConstraint(slowRegionBlueBump);
    }
    //configure traj with speed and acceleration 
    frc::TrajectoryConfig config{ constants::swerveConstants::MaxSpeed*.25, 
                                  constants::swerveConstants::MaxAcceleration*.15};
    
    units::inch_t RedTapeX = 610.77_in - 14.25_in;
    units::inch_t BlueTapeX = 40.45_in + 14.25_in;

    frc::Pose2d Red1Right{  RedTapeX - constants::swerveConstants::WheelBaseLength/2.0, 
                            42.19_in - 22_in, 
                            frc::Rotation2d(180_deg)
                          };
    frc::Pose2d Red1Center{ RedTapeX - constants::swerveConstants::WheelBaseLength/2.0, 
                            42.19_in, 
                            frc::Rotation2d(180_deg)
                          };
    frc::Pose2d Red1Left{ RedTapeX - constants::swerveConstants::WheelBaseLength/2.0, 
                          42.19_in + 22_in, 
                          frc::Rotation2d(180_deg)
                        };

    frc::Pose2d Red2Right{  RedTapeX - constants::swerveConstants::WheelBaseLength/2.0, 
                            108.19_in - 22_in, 
                            frc::Rotation2d(180_deg)
                          };
    frc::Pose2d Red2Center{ RedTapeX - constants::swerveConstants::WheelBaseLength/2.0, 
                            108.19_in, 
                            frc::Rotation2d(180_deg)
                          };
    frc::Pose2d Red2Left{ RedTapeX - constants::swerveConstants::WheelBaseLength/2.0, 
                          108.19_in + 22_in, 
                          frc::Rotation2d(180_deg)
                        };
    
    frc::Pose2d Red3Right{  RedTapeX - constants::swerveConstants::WheelBaseLength/2.0, 
                            147.19_in - 22_in, 
                            frc::Rotation2d(180_deg)
                          };
    frc::Pose2d Red3Center{ RedTapeX - constants::swerveConstants::WheelBaseLength/2.0, 
                            147.19_in, 
                            frc::Rotation2d(180_deg)
                          };
    frc::Pose2d Red3Left{ RedTapeX - constants::swerveConstants::WheelBaseLength/2.0, 
                          147.19_in + 22_in, 
                          frc::Rotation2d(180_deg)
                        };

    frc::Pose2d Blue8Right{ BlueTapeX + constants::swerveConstants::WheelBaseLength/2.0, 
                            42.19_in + 22_in, 
                            frc::Rotation2d(0_deg)
                          };
    frc::Pose2d Blue8Center{ BlueTapeX + constants::swerveConstants::WheelBaseLength/2.0, 
                            42.19_in, 
                            frc::Rotation2d(0_deg)
                          };
    frc::Pose2d Blue8Left{ BlueTapeX + constants::swerveConstants::WheelBaseLength/2.0, 
                          42.19_in - 22_in, 
                          frc::Rotation2d(0_deg)
                        };

    frc::Pose2d Blue7Right{ BlueTapeX + constants::swerveConstants::WheelBaseLength/2.0, 
                            108.19_in + 22_in, 
                            frc::Rotation2d(0_deg)
                          };
    frc::Pose2d Blue7Center{ BlueTapeX + constants::swerveConstants::WheelBaseLength/2.0, 
                            108.19_in, 
                            frc::Rotation2d(0_deg)
                          };
    frc::Pose2d Blue7Left{ BlueTapeX + constants::swerveConstants::WheelBaseLength/2.0, 
                          108.19_in - 22_in, 
                          frc::Rotation2d(0_deg)
                        };
    
    frc::Pose2d Blue6Right{ BlueTapeX + constants::swerveConstants::WheelBaseLength/2.0, 
                            147.19_in + 22_in, 
                            frc::Rotation2d(0_deg)
                          };
    frc::Pose2d Blue6Center{ BlueTapeX + constants::swerveConstants::WheelBaseLength/2.0, 
                            147.19_in, 
                            frc::Rotation2d(0_deg)
                          };
    frc::Pose2d Blue6Left{ BlueTapeX + constants::swerveConstants::WheelBaseLength/2.0, 
                          147.19_in - 22_in, 
                          frc::Rotation2d(0_deg)
                        };
    

    frc::Translation2d BlueCorridorNear{ 2.2_m, 4.7_m}; //waypoing near scoring zone for the flat corridor to game pieces
    frc::Translation2d BlueCorridorFar{  5.6_m, 4.7_m}; //waypoing near game pieces for the flat corridor to game pieces
    frc::Translation2d BlueBumpNear{ 2.2_m, 0.75_m};     //waypoing near scoring zone for the Speed Bump to game pieces
    frc::Translation2d BlueBumpFar{  5.6_m, 0.75_m};     //waypoing near game pieces for the Speed Bump to game pieces

    frc::Translation2d BlueSwitchNear{ BlueTapeX + 30_in, 107.39_in};     //waypoing near alliance wall for switch
    frc::Translation2d BlueSwitch{     BlueTapeX + 96.75_in, 107.39_in}; //Center of switch
    frc::Translation2d BlueSwitchFar{  BlueTapeX + 162_in, 107.39_in};     //waypoing Far from alliance wall for switch

    frc::Translation2d BluePiece1{ BlueTapeX + 224_in, 180.19_in};  //farthest from wall
    frc::Translation2d BluePiece2{ BlueTapeX + 224_in, 132.19_in};
    frc::Translation2d BluePiece3{ BlueTapeX + 224_in,  84.19_in};
    frc::Translation2d BluePiece4{ BlueTapeX + 224_in,  36.19_in};  //nearest to wall

    frc::Translation2d RedCorridorNear{ 14.3_m, 4.7_m}; //waypoing near scoring zone for the flat corridor to game pieces
    frc::Translation2d RedCorridorFar{  11_m, 4.7_m};   //waypoing near game pieces for the flat corridor to game pieces
    frc::Translation2d RedBumpNear{ 14.3_m, 0.75_m};    //waypoing near scoring zone for the Speed Bump to game pieces
    frc::Translation2d RedBumpFar{  11_m, 0.75_m};      //waypoing near game pieces for the Speed Bump to game pieces
    
    frc::Translation2d RedSwitchNear{ RedTapeX - 30_in, 107.39_in};     //waypoing near alliance wall for switch
    frc::Translation2d RedSwitch{     RedTapeX - 96.75_in, 107.39_in}; //Center of switch
    frc::Translation2d RedSwitchFar{  RedTapeX - 162_in, 107.39_in};     //waypoing Far from alliance wall for switch

    frc::Translation2d RedPiece1{ RedTapeX - 224_in, 180.19_in};
    frc::Translation2d RedPiece2{ RedTapeX - 224_in, 132.19_in};
    frc::Translation2d RedPiece3{ RedTapeX - 224_in,  84.19_in};
    frc::Translation2d RedPiece4{ RedTapeX - 224_in,  36.19_in};


            
};
