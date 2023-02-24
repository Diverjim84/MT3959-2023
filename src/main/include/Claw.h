// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
#pragma once

#include "LoggingLevel.h"
#include <units/math.h>
#include <wpi/sendable/Sendable.h>
#include <ctre/Phoenix.h>

class Claw {

TalonFX m_motorAngular; //declares angular motor
TalonFX m_motorIntake; //declares intake motor
CANCoder m_encoder; //declares encoder

    public:

void Init();

void SetIntakeSpeed(); //sets intake motor speed
void SetAngularSpeed(); //sets angular motor speed
void SetClawPosition(); //sets claw position

void SendData(LoggingLevel verbose); //sends data to dash board

void GetIntakeSpeed(); //returns intake speed
void GetRawIntakeSpeed(); //returns raw intake speed
void GetAngularSpeed(); //return angular motor speed
void GetRawAngularSpeed(); //returns raw angular speed
void GetTargetPosition(); //returns target position
void GetError(); //returns error
void GetRawError(); //returns raw error
};