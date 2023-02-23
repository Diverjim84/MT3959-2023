#include "Slide.h"

void Slide::Init(){
    //set motors and encoders
}

void Slide::SetPosition(){
    //sets position
}

void Slide::SetSpeed(double speed){
    //sets the speed of the motor
}

void Slide::GetPosition(){
    //returns position
}

void Slide::GetRawPosition(){
    //returns raw position
}

void Slide::GetSpeed(){
    //returns speed of motor
}

void Slide::GetRawSpeed(){
    //returns raw speed of motor
}

void Slide::GetTargetPosition(){
    //returns the desired position(Forward or back)
}

void Slide::GetError(){
    //detirmins and returns the error
}

void Slide::GetRawError(){
    //returns raw error
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