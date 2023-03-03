#include "MTechArm.h" //header file

Arm::Arm(ArmConstants constants): 
      m_encoder(constants.encoderID),
      m_motor1(constants.motor1ID),
      m_motor2(constants.motor2ID)
      

{
    //constructor- declaring constants for arm
}

void Arm::Init(ArmConstants constants){
//initialization for the code
    m_motor2.Follow(m_motor1); //slave follow

    //inverting motor
    m_motor1.SetInverted(TalonFXInvertType::Clockwise);    
    m_motor2.SetInverted(TalonFXInvertType::CounterClockwise);

    //zeroing the sensors
    m_motor1.SetSelectedSensorPosition(0);
    m_motor2.SetSelectedSensorPosition(0); 
   
}

void Arm::configDevices(){
    //config for motors and encoders
    //resets
    m_motor1.ConfigFactoryDefault();
    m_motor2.ConfigFactoryDefault();
    m_encoder.ConfigFactoryDefault();

    TalonFXConfiguration config;

    config.slot0.kP = .01;
    config.slot0.kF = .05;

    double degreesPerSec = 1.0; 

    config.motionCruiseVelocity = (degreesPerSec / 10.0) * (2048.0 * constants::armConstants::TurnGearRatio / 360.0);
    config.motionAcceleration = config.motionCruiseVelocity; // 1 sec for arm to achieve cruising velocity

    //config.Motor1Config.initializationStrategy = phoenix::sensors::SensorInitializationStrategy::BootToZero;
    m_motor1.ConfigAllSettings(config);
    m_motor2.ConfigAllSettings(config);

    CANCoderConfiguration eConfig;

    eConfig.magnetOffsetDegrees = 0.0;
    m_encoder.ConfigAllSettings(eConfig);

}

void Arm::SetAngle(units::degree_t goal){
    double deltaDeg = frc::Rotation2d(goal-GetAngle()).Degrees().value(); //how far away goal is from now

    double turnTarget = m_motor1.GetSelectedSensorPosition()+deltaDeg*(2048.0*constants::armConstants::TurnGearRatio/360.0); //how many ticks away the goal is

    m_motor1.Set(motorcontrol::ControlMode::MotionMagic, turnTarget); //tells motor controller to go to turnTarget with MotionMagic
    //closed loop set point function
} 

void Arm::SetSpeed(double rawMotorSpeed){
    m_motor1.Set(motorcontrol::ControlMode::PercentOutput, rawMotorSpeed);
    m_motor2.Set(motorcontrol::ControlMode::PercentOutput, rawMotorSpeed); //sets motors to % output motor control
} 

units::degree_t Arm::GetOffsetAngle(){
    double encoderRaw = m_encoder.ConfigGetParameter(ParamEnum::eMagnetOffset, 0); //offset angle
    units::degree_t angle(encoderRaw); //convert to degrees
    return angle;
}

units::degree_t Arm::GetAngle(){
    double encoderRaw = m_encoder.GetAbsolutePosition(); //gets the angle the arm is currently at
    units::degree_t angle(encoderRaw); //convert to degrees
    return angle; 
}

double Arm::GetRawAngle(){
    double encoderRaw = m_encoder.GetAbsolutePosition(); //gets the angle without being zeroed out - encoder ticks
    return encoderRaw;
}


void Arm::SendData(LoggingLevel verbose){
    //sends data to dashboard with the enum LoggingLevel
    switch(verbose){
        case LoggingLevel::Everything: //everything that is not in the cases below it
                                    //continue
                                    {
                                        frc::SmartDashboard::PutNumber(" heading", GetAngle().value());
                                        frc::SmartDashboard::PutNumber(" raw turn encoder position", m_encoder.GetAbsolutePosition());
                                    }
        case LoggingLevel::PID: //send PID (closed loop control) data
                                    //continue
                                    {
                                        frc::SmartDashboard::PutNumber(" raw turn pid target", ctreHelpers::CTRE_Get_PID_Target(m_motor1)); //target of motor
                                        frc::SmartDashboard::PutNumber(" raw turn pid error", ctreHelpers::CTRE_Get_PID_Error(m_motor1)); //error 
                                    }
                                    //continue
        case LoggingLevel::Basic: //minimal useful data to driver
                                    //continue
                                    {
                                            frc::SmartDashboard::PutNumber(" arm speed", m_motor1.GetSelectedSensorVelocity());
                                            frc::SmartDashboard::PutNumber(" raw arm position", m_motor1.GetSelectedSensorPosition());
                                    }
        default: break; //make sure nothing else prints
        

    }
}


