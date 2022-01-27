package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.sensors.CANCoder;

import static frc.robot.Constants.*;

public class ClimbSubsystem extends SubsystemBase 
{
    TalonSRX m_climb6Motor = new TalonSRX(CLIMB_6_MOTOR); 
    TalonSRX m_climb10Motor = new TalonSRX(CLIMB_10_MOTOR); 
    TalonSRX m_climb15Motor = new TalonSRX(CLIMB_15_MOTOR); 
    CANCoder m_climb6Encoder = new  CANCoder(CLIMB_6_ENCODER);
    CANCoder m_climb10Encoder = new  CANCoder(CLIMB_10_ENCODER);
    CANCoder m_climb15Encoder = new  CANCoder(CLIMB_15_ENCODER);


    public void setClimb6Speed(double value){
        m_climb6Motor.set(ControlMode.PercentOutput, value);
    }


    public void setClimb10Speed(double value){
        m_climb10Motor.set(ControlMode.PercentOutput, value);
    }    

    
    public void setClimb15Speed(double value){
        m_climb15Motor.set(ControlMode.PercentOutput, value);
    }
}

