#include "Elevator.h" //header file


Elevator::Elevator()
{
    //constructor- declaring constants for elevator
    Init();

}


void Elevator::Init(){
//initialization for the code

    m_motor2.Follow(m_motor1); //slave follow

    //inverting motor
    m_motor1.SetInverted(TalonFXInvertType::Clockwise);    
    m_motor2.SetInverted(m_motor1.GetInverted());

    //zeroing the sensors
    m_motor1.SetSelectedSensorPosition(0);
    m_motor2.SetSelectedSensorPosition(0); 
   
}
void Elevator::configDevices(){

}
      

void Elevator::SetSpeed(double rawMotorSpeed){
    m_motor1.Set(motorcontrol::ControlMode::PercentOutput, rawMotorSpeed);
    m_motor2.Set(motorcontrol::ControlMode::PercentOutput, rawMotorSpeed); //sets motors to % output motor control
} 

void Elevator::SendData(LoggingLevel verbose){
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
