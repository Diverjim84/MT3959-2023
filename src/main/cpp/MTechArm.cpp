#include "MTechArm.h" //header file

Arm::Arm(ArmConstants constants)
    : m_motor1(constants.motor1ID),
      m_motor2(constants.motor2ID),
      m_encoder(constants.encoderID)

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
    //sets motors to % output motor control
} 

units::degree_t Arm::GetOffsetAngle(){
    //double to get raw encoder value, then convert to degrees
    return 0_deg;
}

units::degree_t Arm::GetAngle(){
    return 0_deg; 
}

units::degree_t Arm::GetTargetAngle(){
    return 0_deg;
}

double Arm::GetRawAngle(){
    return 0.0;
}

double Arm::GetAngleError(){
   //angles 
   return 0.0;
} 

units::degrees_per_second_t Arm::GetSpeed(){
    return 0_deg_per_s;
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