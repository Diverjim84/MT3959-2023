#include "MTechArm.h" //header file

Arm::Arm(ArmConstants constants): 
      m_encoder(constants.encoderID),
      m_motor1(constants.motor1ID),
      m_motor2(constants.motor2ID)
      

{
    //constructor- declaring constants for arm
    //Arm::x used since method is outside of the arm class
}

void Arm::Init(ArmConstants constants){
//initialization for the code
    //set motors and encoders
    //config settings for motors and encoder
    //zero the sensors
    
}

void Arm::SetAngle(){
    
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

units::degree_t Arm::GetTargetAngle(){
    double motor1 = ctreHelpers::CTRE_Get_PID_Target(m_motor1); //sends the angle the arm intended to go to - easier to calculate error
    units::degree_t angle(motor1);
    return angle;
}

double Arm::GetRawAngle(){
    double encoderRaw = m_encoder.GetAbsolutePosition(); //gets the angle without being zeroed out - encoder ticks
    return encoderRaw;
}

double Arm::GetAngleError(){
   //how off the intended angle is to the actual angle (encoder ticks)
   return 0.0;
} 

units::degrees_per_second_t Arm::GetSpeed(){
    double encoderRaw = m_encoder.GetVelocity();
    units::degrees_per_second_t speed(encoderRaw);
    //gets the speed of the arm in degrees per second as it rotates
    return speed;
}

double Arm::GetMotorEncoderPosition(){
    
    return 0.0;
}

void Arm::SendData(LoggingLevel verbose){
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


//replaced ; with {}