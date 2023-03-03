#include "Slide.h"

void Slide::Init(){
    //set motors and encoders
}

void Slide::Config(){
    m_motor.ConfigFactoryDefault();
    m_encoder.ConfigFactoryDefault();

    TalonFXConfiguration config;

    config.slot0.kP = .01;
    config.slot0.kF = .05;

    double inchesPerSec = 0.25; 

    config.motionCruiseVelocity = (inchesPerSec / 10.0) * (2048.0 * constants::slideConstants::MotorGearRatio / 360.0);
    config.motionAcceleration = config.motionCruiseVelocity; // 1 sec for arm to achieve cruising velocity

    //config.Motor1Config.initializationStrategy = phoenix::sensors::SensorInitializationStrategy::BootToZero;
    m_motor.ConfigAllSettings(config);

    CANCoderConfiguration eConfig;

    eConfig.magnetOffsetDegrees = 0.0;
    m_encoder.ConfigAllSettings(eConfig);

}

void Slide::SetPosition(units::inch_t position){
    //sets position
}

void Slide::SetSpeed(double speed){
    //sets the speed of the motor
}

units::inch_t Slide::GetPosition(){
    //returns position
}

void Slide::SendData(LoggingLevel verbose){
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