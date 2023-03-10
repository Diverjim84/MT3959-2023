
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
#include "Claw.h"

void Claw::Init(){
    //sets motors and encoders
    m_motorRightIntake.SetInverted(false);
    m_motorLeftIntake.SetInverted(!m_motorRightIntake.GetInverted());
}

void Claw::SetIntakeSpeed(double Speed){
    //sets intake motor speed
    m_motorRightIntake.Set(Speed);
    m_motorLeftIntake.Set(Speed);
}

void Claw::ClawOpen(){
    m_clawPistion.Set(frc::DoubleSolenoid::Value::kForward);
}

void Claw::ClawClose(){
    m_clawPistion.Set(frc::DoubleSolenoid::Value::kReverse);
}

void Claw::SendData(LoggingLevel verbose){
    //sends data to dashboard with the enum LoggingLevel
    switch(verbose){
        case LoggingLevel::Everything: //everything that is not in the cases below it
                                    //continue
                                    {
                                        frc::SmartDashboard::PutNumber("Claw position", m_clawPistion.Get());
                                        frc::SmartDashboard::PutNumber("Claw right", m_motorRightIntake.Get());
                                        frc::SmartDashboard::PutNumber("Claw left", m_motorLeftIntake.Get());
                                    }
        case LoggingLevel::PID: //send PID (closed loop control) data
                                    //continue
        case LoggingLevel::Basic: //minimal useful data to driver
                                    //continue
        default: break; //make sure nothing else prints
        

    }
}