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

    public void ClimbSubsystemInit(){
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
        m_climb6Motor.configPeakCurrentLimit(30, Constants.TALON_TIMEOUT_MS);
        m_climb6Motor.configPeakCurrentDuration(200, Constants.TALON_TIMEOUT_MS);
        m_climb6Motor.configContinuousCurrentLimit(25, Constants.TALON_TIMEOUT_MS);
        m_climb6Motor.enableCurrentLimit(true);
        // Set Motion Magic gains in slot0
        m_climb6Motor.selectProfileSlot(0, 0);
        m_climb6Motor.config_kF(0, Constants.CLIMB6_MOTOR_KF, Constants.TALON_TIMEOUT_MS);
        m_climb6Motor.config_kP(0, Constants.CLIMB6_MOTOR_KP, Constants.TALON_TIMEOUT_MS);
        m_climb6Motor.config_kI(0, Constants.CLIMB6_MOTOR_KI, Constants.TALON_TIMEOUT_MS);
        m_climb6Motor.config_kD(0, Constants.CLIMB6_MOTOR_KD, Constants.TALON_TIMEOUT_MS);
        // Set acceleration and cruise velocity
        m_climb6Motor.configMotionCruiseVelocity(1000, Constants.TALON_TIMEOUT_MS);
        m_climb6Motor.configMotionAcceleration(1000, Constants.TALON_TIMEOUT_MS);
        //  Zero the sensor once on robot startup
        m_climb6Motor.setSelectedSensorPosition(CLIMB6_POSITION_IN, 0, Constants.TALON_TIMEOUT_MS);
        //m_climb6Motor.set(ControlMode.MotionMagic, CLIMB6_POSITION_IN);
    }
    
    public void setClimb6Speed(double value){
        if(Math.abs(value) >= 0.4){
            m_climb6Motor.set(ControlMode.PercentOutput, value);
        }else{
            //m_climb6Motor.set(ControlMode.PercentOutput, 0);
        }
    }

    public void setClimb10Speed(double value){
        //m_climb10Motor.set(ControlMode.PercentOutput, value);
        if (value >= 0.4){
            m_climb6Motor.set(ControlMode.MotionMagic, 10000);
        }else if (value<-0.4){
            m_climb6Motor.set(ControlMode.MotionMagic, 0);
        }
        else{
             m_climb6Motor.set(ControlMode.PercentOutput, 0);
        }
    }    

    public void setClimb15Speed(double value){
        m_climb15Motor.set(ControlMode.PercentOutput, value);
    }
}

