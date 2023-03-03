


#pragma once

#include "LoggingLevel.h"
#include <units/math.h>
#include <wpi/sendable/Sendable.h>
#include <ctre/Phoenix.h>
#include "Constants.h"

class Slide {

TalonFX m_motor{5}; //declares motor
CANCoder m_encoder{6}; //declares encoder


public:
    
    void Init();
    void Config();

    void SetSpeed(double MotorSpeed); //sets % motor speed
    void SetPosition(units::inch_t position); // sets motor position between forward or back

    void SlideForward();
    void SlideBack();

    void SendData(LoggingLevel verbose); //sends data to dash board

    units::inch_t GetPosition(); //returns position
};