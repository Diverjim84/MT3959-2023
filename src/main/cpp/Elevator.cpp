#include "Elevator.h" //header file


Elevator::Elevator(constants::elevatorConstants::elevatorMotors constants): 
      m_encoder(constants.encoder_ID),
      m_motor11(constants.motor1_ID),
      m_motor12(constants.motor2_ID)

{
    //constructor- declaring constants for arm
}


void Elevator::Init(constants::elevatorConstants::elevatorMotors constants){
//initialization for the code
    m_motor12.Follow(m_motor11); //slave follow

    //inverting motor
    m_motor11.SetInverted(TalonFXInvertType::Clockwise);    
    m_motor12.SetInverted(TalonFXInvertType::CounterClockwise);

    //zeroing the sensors
    m_motor11.SetSelectedSensorPosition(0);
    m_motor12.SetSelectedSensorPosition(0); 
   
}

      

void Elevator::SetSpeed(double rawMotorSpeed){
    m_motor11.Set(motorcontrol::ControlMode::PercentOutput, rawMotorSpeed);
    m_motor12.Set(motorcontrol::ControlMode::PercentOutput, rawMotorSpeed); //sets motors to % output motor control
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
