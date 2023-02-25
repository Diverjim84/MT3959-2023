#pragma once

#include <units/angle.h>
#include <units/math.h>
#include <units/angular_velocity.h>
#include <wpi/sendable/Sendable.h>
#include <ctre/Phoenix.h>

#include "LoggingLevel.h"
#include "MTechArmConstants.h"
#include "Constants.h"
#include "CTREHelpers.h"


class Arm : public wpi::Sendable /*constructor*/{
private:
    bool m_angleOffset; //encoder offset, zeroes the value
    CANCoder m_encoder; //declares encoder

    TalonFX m_motor1; //declares primary motor
    TalonFX m_motor2; //declares follower motor

    units::degree_t m_targetAngle; //shows the target angle of the motors/arm in degrees

    void configDevices();
    
public:
    

    Arm(ArmConstants constants);
    void Init(ArmConstants constants);

    void SetAngle(units::degree_t goal); //closed loop set point function
    void SetSpeed(double rawMotorSpeed); //sets motors to % output motor control

    void SendData(LoggingLevel verbose); //sends LoggingLevel data to dashboard
    units::degree_t GetOffsetAngle(); //gives the offset of the angle in degrees
    units::degree_t GetAngle(); //gives the current angle in degrees
    double GetRawAngle(); //sends the raw angle w/o zeroing it out
    };