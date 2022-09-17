package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import com.ctre.phoenix.motorcontrol.can.*;
import com.ctre.phoenix.ErrorCode;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import static frc.robot.Constants.*;

public class IntakeSubsystem extends SubsystemBase {
    TalonSRX m_intakeArmMotor = new TalonSRX(INTAKE_ARM_MOTOR); 
    TalonSRX m_intakeMotor = new TalonSRX(INTAKE_MOTOR); 
    

    public void IntakeSubsystemInit() {
        // Set peak current
        m_intakeMotor.configPeakCurrentLimit(12, Constants.TALON_TIMEOUT_MS);
        m_intakeMotor.configPeakCurrentDuration(200, Constants.TALON_TIMEOUT_MS);
        m_intakeMotor.configContinuousCurrentLimit(10, Constants.TALON_TIMEOUT_MS);
        m_intakeMotor.enableCurrentLimit(true);

        m_intakeArmMotor.configFactoryDefault();
        m_intakeArmMotor.configSelectedFeedbackSensor( FeedbackDevice.CTRE_MagEncoder_Relative, 0, Constants.TALON_TIMEOUT_MS);
        m_intakeArmMotor.configNeutralDeadband(0.10, Constants.TALON_TIMEOUT_MS);
        // Configure Talon  SRX output and sensor direction
        m_intakeArmMotor.setSensorPhase(false);
        // Set peak current
        m_intakeArmMotor.configContinuousCurrentLimit(6, Constants.TALON_TIMEOUT_MS);
        m_intakeArmMotor.configPeakCurrentLimit(0, Constants.TALON_TIMEOUT_MS);
        m_intakeArmMotor.enableCurrentLimit(true);
        // Set Motion Magic gains in slot0
        m_intakeArmMotor.selectProfileSlot(0, 0);
        m_intakeArmMotor.config_kF(0, Constants.INTAKE_ARM_MOTOR_KF, Constants.TALON_TIMEOUT_MS);
        m_intakeArmMotor.config_kP(0, Constants.INTAKE_ARM_MOTOR_KP, Constants.TALON_TIMEOUT_MS);
        m_intakeArmMotor.config_kI(0, Constants.INTAKE_ARM_MOTOR_KI, Constants.TALON_TIMEOUT_MS);
        m_intakeArmMotor.config_kD(0, Constants.INTAKE_ARM_MOTOR_KD, Constants.TALON_TIMEOUT_MS);
        // Set acceleration and cruise velocity
        m_intakeArmMotor.configMotionCruiseVelocity(1000, Constants.TALON_TIMEOUT_MS);
        m_intakeArmMotor.configMotionAcceleration(1000, Constants.TALON_TIMEOUT_MS);
        //  Zero the sensor once on robot startup
        m_intakeArmMotor.setSelectedSensorPosition(0, 0, Constants.TALON_TIMEOUT_MS);
        m_intakeArmMotor.set( ControlMode.MotionMagic, INTAKE_ARM_POSITION_IN_FULL );
    }
    public void setIntakeSpeed(double value){
        m_intakeMotor.set(ControlMode.PercentOutput, value);
    }

    public void setIntakeArmPosition(double value){
        m_intakeArmMotor.set(ControlMode.MotionMagic, value);
    }
}