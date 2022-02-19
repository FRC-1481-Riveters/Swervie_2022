package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import static frc.robot.Constants.*;

public class IntakeSubsystem extends SubsystemBase {
    TalonSRX m_intakeArmMotor = new TalonSRX(INTAKE_ARM_MOTOR); 
    TalonSRX m_intakeMotor = new TalonSRX(INTAKE_MOTOR); 

    public void setIntakeSpeed(double value){
        m_intakeMotor.set(ControlMode.PercentOutput, value);
    }

    public void setIntakeArmSpeed(double value){
        m_intakeArmMotor.set(ControlMode.PercentOutput, value);
    }
}