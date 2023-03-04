#pragma once

#include <units/angle.h>
#include <units/math.h>
#include <units/angular_velocity.h>
#include <wpi/sendable/Sendable.h>
#include <ctre/Phoenix.h>
#include <frc/smartdashboard/SmartDashboard.h>

#include "LoggingLevel.h"
#include "Constants.h"
#include "CTREHelpers.h"

class Elevator : public wpi::Sendable /*constructor*/{
private:
    CANCoder m_encoder; //declares encoder

    TalonFX m_motor11; //declares master motor
    TalonFX m_motor12; //declares slave motor

public:
    
    
    Elevator(constants::elevatorConstants::elevatorMotors constants);
    void Init(constants::elevatorConstants::elevatorMotors constants);
 

    void SetSpeed(double rawMotorSpeed); //sets motors to % output motor control
    
    void SendData(LoggingLevel verbose); //sends LoggingLevel data to dashboard
    units::degrees_per_second_t GetSpeed(); //sends the speed of the motors in deg/sec
};