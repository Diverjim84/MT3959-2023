


#pragma once

#include "LoggingLevel.h"
#include <units/math.h>
#include <wpi/sendable/Sendable.h>
#include <ctre/Phoenix.h>

class Slide {

TalonFX m_motor; //declares motor
CANCoder m_encoder; //declares encoder


    public:
    
    void Init();

    void SetSpeed(double MotorSpeed); //sets % motor speed
    void SetPosition(); // sets motor position between forward or back

    void GetPosition(); //returns position
    void GetRawPosition(); //returns raw position
    void GetSpeed(); //returns speed
    void GetRawSpeed(); //returns raw speed
    void GetTargetPosition(); //returns target position
    void GetError(); //returns error
    void GetRawError(); //returns error
};