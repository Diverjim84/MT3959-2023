#pragma once

#include <units/angle.h>
#include <units/math.h>
#include <units/angular_velocity.h>
#include <wpi/sendable/Sendable.h>
#include <ctre/Phoenix.h>

#include "LoggingLevel.h"
#include "MTechArmConstants.h"
#include "CTREHelpers.h"


class Arm : public wpi::Sendable /*constructor*/{
private:
    bool m_angleOffset; //encoder offset, zeroes the value
    CANCoder m_encoder; //declares encoder

    TalonFX m_motor1; //declares primary motor
    TalonFX m_motor2; //declares follower motor

    units::degree_t m_targetAngle; //shows the target angle of the motors/arm in degrees
    
public:
    

    Arm(ArmConstants constants);
    void Init(ArmConstants constants);

    void SetAngle(); //closed loop set point function
    void SetSpeed(double rawMotorSpeed); //sets motors to % output motor control

    void SendData(LoggingLevel verbose); //sends LoggingLevel data to dashboard
    units::degree_t GetOffsetAngle(); //gives the offset of the angle in degrees
    units::degree_t GetAngle(); //gives the current angle in degrees
    units::degree_t GetTargetAngle(); //gives the target angle in degrees
    double GetRawAngle(); //sends the raw angle w/o zeroing it out
    double GetAngleError(); //sends the error from the raw angle to the true angle (zeroed out)
    units::degrees_per_second_t GetSpeed(); //sends the speed of the motors in deg/sec
    double GetMotorEncoderPosition(); //sends raw encoder position of motor
};