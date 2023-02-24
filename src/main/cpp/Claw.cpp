
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
#include "Claw.h"

void Claw::Init(){
    //sets motors and encoders
}

void Claw::SetIntakeSpeed(){
    //sets intake motor speed
}

void Claw::SetAngularSpeed(){
    //sets angular motor speed
}

void Claw::SetClawPosition(){
    //sets claw intake type
}

void Claw::GetIntakeSpeed(){
    //returns intake motor speed
}

void Claw::GetRawIntakeSpeed(){
    //returns raw intake motor speed
}

void Claw::GetAngularSpeed(){
    //returns angular motor speed
}

void Claw::GetRawAngularSpeed(){
    //returns raw angular motor speed
}

void Claw::GetTargetPosition(){
    //returns target intake position
}

void Claw::GetError(){
    //determines and returns error
}

void Claw::GetRawError(){
    //returns raw error
}

void Claw::SendData(LoggingLevel verbose){
    //sends data to dashboard with the enum LoggingLevel
    switch(verbose){
        case LoggingLevel::Everything: //everything that is not in the cases below it
                                    //continue
        case LoggingLevel::PID: //send PID (closed loop control) data
                                    //continue
        case LoggingLevel::Basic: //minimal useful data to driver
                                    //continue
        default: break; //make sure nothing else prints
        

    }
}