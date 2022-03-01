package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import static frc.robot.Constants.*;

public class ClimbSubsystem extends SubsystemBase {
    TalonSRX m_climb6Motor = new TalonSRX(CLIMB_6_MOTOR); 
    TalonSRX m_climb10Motor = new TalonSRX(CLIMB_10_MOTOR); 
    TalonSRX m_climb15Motor = new TalonSRX(CLIMB_15_MOTOR); 
    CANCoder m_climb6Encoder = new  CANCoder(CLIMB_6_ENCODER);
    CANCoder m_climb10Encoder = new  CANCoder(CLIMB_10_ENCODER);
    CANCoder m_climb15Encoder = new  CANCoder(CLIMB_15_ENCODER);

    public void ClimbSubsystemInit() {

        m_climb6Motor.configFactoryDefault();
        m_climb6Motor.configSelectedFeedbackSensor( FeedbackDevice.CTRE_MagEncoder_Relative, 0, Constants.TALON_TIMEOUT_MS);
        m_climb6Motor.configNeutralDeadband(0.10, Constants.TALON_TIMEOUT_MS);
        // Configure Talon  SRX output and sensor direction
        m_climb6Motor.setSensorPhase(true);
        m_climb6Motor.setInverted(false);
        // Set relevant frame periods to be at least twice as fast as periodic rate
        m_climb6Motor.setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0, 10, Constants.TALON_TIMEOUT_MS);
        m_climb6Motor.setStatusFramePeriod(StatusFrameEnhanced.Status_10_MotionMagic, 10, Constants.TALON_TIMEOUT_MS);
        // Set the peak and nominal outputs
        m_climb6Motor.configNominalOutputForward(0, Constants.TALON_TIMEOUT_MS);
        m_climb6Motor.configNominalOutputReverse(0, Constants.TALON_TIMEOUT_MS);
        m_climb6Motor.configPeakOutputForward(1, Constants.TALON_TIMEOUT_MS);
        m_climb6Motor.configPeakOutputReverse(-1, Constants.TALON_TIMEOUT_MS);
        // Set peak current
        m_climb6Motor.configPeakCurrentLimit(35, Constants.TALON_TIMEOUT_MS);
        m_climb6Motor.configPeakCurrentDuration(200, Constants.TALON_TIMEOUT_MS);
        m_climb6Motor.configContinuousCurrentLimit(30, Constants.TALON_TIMEOUT_MS);
        m_climb6Motor.enableCurrentLimit(true);
        // Set Motion Magic gains in slot0
        m_climb6Motor.selectProfileSlot(0, 0);
        m_climb6Motor.config_kF(0, Constants.CLIMB6_MOTOR_KF, Constants.TALON_TIMEOUT_MS);
        m_climb6Motor.config_kP(0, Constants.CLIMB6_MOTOR_KP, Constants.TALON_TIMEOUT_MS);
        m_climb6Motor.config_kI(0, Constants.CLIMB6_MOTOR_KI, Constants.TALON_TIMEOUT_MS);
        m_climb6Motor.config_kD(0, Constants.CLIMB6_MOTOR_KD, Constants.TALON_TIMEOUT_MS);
        // Set acceleration and cruise velocity
        m_climb6Motor.configMotionCruiseVelocity(1500, Constants.TALON_TIMEOUT_MS);
        m_climb6Motor.configMotionAcceleration(3000, Constants.TALON_TIMEOUT_MS);
        //  Zero the sensor once on robot startup
        m_climb6Motor.setSelectedSensorPosition(CLIMB6_POSITION_IN, 0, Constants.TALON_TIMEOUT_MS);

        m_climb10Motor.configFactoryDefault();
        m_climb10Motor.configSelectedFeedbackSensor( FeedbackDevice.CTRE_MagEncoder_Relative, 0, Constants.TALON_TIMEOUT_MS);
        m_climb10Motor.configNeutralDeadband(0.10, Constants.TALON_TIMEOUT_MS);
        // Configure Talon  SRX output and sensor direction
        m_climb10Motor.setSensorPhase(true);
        m_climb10Motor.setInverted(false);
        // Set relevant frame periods to be at least twice as fast as periodic rate
        m_climb10Motor.setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0, 10, Constants.TALON_TIMEOUT_MS);
        m_climb10Motor.setStatusFramePeriod(StatusFrameEnhanced.Status_10_MotionMagic, 10, Constants.TALON_TIMEOUT_MS);
        // Set the peak and nominal outputs
        m_climb10Motor.configNominalOutputForward(0, Constants.TALON_TIMEOUT_MS);
        m_climb10Motor.configNominalOutputReverse(0, Constants.TALON_TIMEOUT_MS);
        m_climb10Motor.configPeakOutputForward(1, Constants.TALON_TIMEOUT_MS);
        m_climb10Motor.configPeakOutputReverse(-1, Constants.TALON_TIMEOUT_MS);
        // Set peak current
        m_climb10Motor.configPeakCurrentLimit(35, Constants.TALON_TIMEOUT_MS);
        m_climb10Motor.configPeakCurrentDuration(200, Constants.TALON_TIMEOUT_MS);
        m_climb10Motor.configContinuousCurrentLimit(30, Constants.TALON_TIMEOUT_MS);
        m_climb10Motor.enableCurrentLimit(true);
        // Set Motion Magic gains in slot0
        m_climb10Motor.selectProfileSlot(0, 0);
        m_climb10Motor.config_kF(0, Constants.CLIMB6_MOTOR_KF, Constants.TALON_TIMEOUT_MS);
        m_climb10Motor.config_kP(0, Constants.CLIMB6_MOTOR_KP, Constants.TALON_TIMEOUT_MS);
        m_climb10Motor.config_kI(0, Constants.CLIMB6_MOTOR_KI, Constants.TALON_TIMEOUT_MS);
        m_climb10Motor.config_kD(0, Constants.CLIMB6_MOTOR_KD, Constants.TALON_TIMEOUT_MS);
        // Set acceleration and cruise velocity
        m_climb10Motor.configMotionCruiseVelocity(1500, Constants.TALON_TIMEOUT_MS);
        m_climb10Motor.configMotionAcceleration(3000, Constants.TALON_TIMEOUT_MS);
        //  Zero the sensor once on robot startup
        m_climb10Motor.setSelectedSensorPosition(CLIMB6_POSITION_IN, 0, Constants.TALON_TIMEOUT_MS);

        m_climb15Motor.configFactoryDefault();
        m_climb15Motor.configSelectedFeedbackSensor( FeedbackDevice.CTRE_MagEncoder_Relative, 0, Constants.TALON_TIMEOUT_MS);
        m_climb15Motor.configNeutralDeadband(0.10, Constants.TALON_TIMEOUT_MS);
        // Configure Talon  SRX output and sensor direction
        m_climb15Motor.setSensorPhase(true);
        m_climb15Motor.setInverted(true);
        // Set relevant frame periods to be at least twice as fast as periodic rate
        m_climb15Motor.setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0, 10, Constants.TALON_TIMEOUT_MS);
        m_climb15Motor.setStatusFramePeriod(StatusFrameEnhanced.Status_10_MotionMagic, 10, Constants.TALON_TIMEOUT_MS);
        // Set the peak and nominal outputs
        m_climb15Motor.configNominalOutputForward(0, Constants.TALON_TIMEOUT_MS);
        m_climb15Motor.configNominalOutputReverse(0, Constants.TALON_TIMEOUT_MS);
        m_climb15Motor.configPeakOutputForward(1, Constants.TALON_TIMEOUT_MS);
        m_climb15Motor.configPeakOutputReverse(-1, Constants.TALON_TIMEOUT_MS);
        // Set peak current
        m_climb15Motor.configPeakCurrentLimit(35, Constants.TALON_TIMEOUT_MS);
        m_climb15Motor.configPeakCurrentDuration(200, Constants.TALON_TIMEOUT_MS);
        m_climb15Motor.configContinuousCurrentLimit(30, Constants.TALON_TIMEOUT_MS);
        m_climb15Motor.enableCurrentLimit(true);
        // Set Motion Magic gains in slot0
        m_climb15Motor.selectProfileSlot(0, 0);
        m_climb15Motor.config_kF(0, Constants.CLIMB6_MOTOR_KF, Constants.TALON_TIMEOUT_MS);
        m_climb15Motor.config_kP(0, Constants.CLIMB6_MOTOR_KP, Constants.TALON_TIMEOUT_MS);
        m_climb15Motor.config_kI(0, Constants.CLIMB6_MOTOR_KI, Constants.TALON_TIMEOUT_MS);
        m_climb15Motor.config_kD(0, Constants.CLIMB6_MOTOR_KD, Constants.TALON_TIMEOUT_MS);
        // Set acceleration and cruise velocity
        m_climb15Motor.configMotionCruiseVelocity(1500, Constants.TALON_TIMEOUT_MS);
        m_climb15Motor.configMotionAcceleration(3000, Constants.TALON_TIMEOUT_MS);
        //  Zero the sensor once on robot startup
        m_climb15Motor.setSelectedSensorPosition(CLIMB6_POSITION_IN, 0, Constants.TALON_TIMEOUT_MS);
    }
    
    public void setClimb6Speed(double value){
        m_climb6Motor.configPeakCurrentLimit(5, Constants.TALON_TIMEOUT_MS);
        m_climb6Motor.configContinuousCurrentLimit(4, Constants.TALON_TIMEOUT_MS);
        m_climb6Motor.set(ControlMode.PercentOutput, value);
    }

    public void setClimb10Speed(double value){
        m_climb10Motor.configPeakCurrentLimit(5, Constants.TALON_TIMEOUT_MS);
        m_climb10Motor.configContinuousCurrentLimit(4, Constants.TALON_TIMEOUT_MS);
        m_climb10Motor.set(ControlMode.PercentOutput, value);
    }

    public void setClimb15Speed(double value){
        m_climb15Motor.configPeakCurrentLimit(5, Constants.TALON_TIMEOUT_MS);
        m_climb15Motor.configContinuousCurrentLimit(4, Constants.TALON_TIMEOUT_MS);
        m_climb15Motor.set(ControlMode.PercentOutput, value);
    }

    public void setClimb6Position(double value){
        m_climb6Motor.configPeakCurrentLimit(35, Constants.TALON_TIMEOUT_MS);
        m_climb6Motor.configContinuousCurrentLimit(30, Constants.TALON_TIMEOUT_MS);
        m_climb6Motor.set(ControlMode.MotionMagic, value);
    }
    public void setClimb10Position(double value){
        m_climb10Motor.configPeakCurrentLimit(35, Constants.TALON_TIMEOUT_MS);
        m_climb10Motor.configContinuousCurrentLimit(30, Constants.TALON_TIMEOUT_MS);
        m_climb10Motor.set(ControlMode.MotionMagic, value);
    }    

    public void setClimb15Position(double value){
        m_climb15Motor.configPeakCurrentLimit(35, Constants.TALON_TIMEOUT_MS);
        m_climb15Motor.configContinuousCurrentLimit(30, Constants.TALON_TIMEOUT_MS);
        m_climb15Motor.set(ControlMode.MotionMagic, value);
    }

    public void zeroClimbSensors(){
        m_climb6Motor.setSelectedSensorPosition(CLIMB6_POSITION_IN, 0, Constants.TALON_TIMEOUT_MS);
        m_climb10Motor.setSelectedSensorPosition(CLIMB6_POSITION_IN, 0, Constants.TALON_TIMEOUT_MS);
        m_climb15Motor.setSelectedSensorPosition(CLIMB6_POSITION_IN, 0, Constants.TALON_TIMEOUT_MS);
    }

    public double getClimb6Position() {
        return m_climb6Motor.getSelectedSensorPosition(0);
    }

    public double getClimb10Position() {
        return m_climb10Motor.getSelectedSensorPosition(0);
    }

    public double getClimb15Position() {
        return m_climb15Motor.getSelectedSensorPosition(0);
    }

}
