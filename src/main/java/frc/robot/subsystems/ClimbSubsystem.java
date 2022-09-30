package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.networktables.NetworkTableEntry;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import static frc.robot.Constants.*;

public class ClimbSubsystem extends SubsystemBase {
    TalonSRX m_climb6Motor = new TalonSRX(CLIMB_6_MOTOR); 
    TalonSRX m_climb10Motor = new TalonSRX(CLIMB_10_MOTOR); 
    TalonSRX m_climb15Motor = new TalonSRX(CLIMB_15_MOTOR); 

    private ShuffleboardTab tab;
    private NetworkTableEntry kP, kI, kD, kCruise, kAcceleration;
    private NetworkTableEntry c15pos, c15set, c10pos, c10set, c6pos, c6set;
    private double c15Position, c10Position, c6Position;

    public void ClimbSubsystemInit() {

        tab = Shuffleboard.getTab("Climb");
        kP = tab.add("kP",CLIMB6_MOTOR_KP).getEntry();
        kI = tab.add("kI",CLIMB6_MOTOR_KI).getEntry();
        kD = tab.add("kD",CLIMB6_MOTOR_KD).getEntry();
        kCruise = tab.add("kCruise",CLIMB6_MOTOR_CRUISE).getEntry();
        kAcceleration = tab.add("kAcceleration",CLIMB6_MOTOR_ACCELERATION).getEntry();
        c15pos = tab.add("c15pos",0).getEntry();
        c15set = tab.add("c15set",0).getEntry();
        c10pos = tab.add("c10pos",0).getEntry();
        c10set = tab.add("c10set",0).getEntry();
        c6pos = tab.add("c6pos",0).getEntry();
        c6set = tab.add("c6set",0).getEntry();
    
        m_climb6Motor.configFactoryDefault();
        m_climb6Motor.configSelectedFeedbackSensor( FeedbackDevice.CTRE_MagEncoder_Relative, 0, Constants.TALON_TIMEOUT_MS);
        m_climb6Motor.configNeutralDeadband(0.10, Constants.TALON_TIMEOUT_MS);
        // Configure Talon  SRX output and sensor direction
        m_climb6Motor.setSensorPhase(true);
        // Set peak current
        m_climb6Motor.configPeakCurrentLimit(35, Constants.TALON_TIMEOUT_MS);
        m_climb6Motor.configPeakCurrentDuration(200, Constants.TALON_TIMEOUT_MS);
        m_climb6Motor.configContinuousCurrentLimit(30, Constants.TALON_TIMEOUT_MS);
        m_climb6Motor.enableCurrentLimit(true);
        // Set Motion Magic gains in slot0
        m_climb6Motor.selectProfileSlot(0, 0);
        m_climb6Motor.config_kF(0, Constants.CLIMB6_MOTOR_KF, Constants.TALON_TIMEOUT_MS);
        m_climb6Motor.config_kP(0, kP.getDouble(Constants.CLIMB6_MOTOR_KP), Constants.TALON_TIMEOUT_MS);
        m_climb6Motor.config_kI(0, kI.getDouble(Constants.CLIMB6_MOTOR_KI), Constants.TALON_TIMEOUT_MS);
        m_climb6Motor.config_kD(0, kD.getDouble(Constants.CLIMB6_MOTOR_KD), Constants.TALON_TIMEOUT_MS);
        // Set acceleration and cruise velocity
        m_climb6Motor.configMotionCruiseVelocity(CLIMB6_MOTOR_CRUISE, Constants.TALON_TIMEOUT_MS);
        m_climb6Motor.configMotionAcceleration(CLIMB6_MOTOR_ACCELERATION, Constants.TALON_TIMEOUT_MS);
        //  Zero the sensor once on robot startup
        m_climb6Motor.setSelectedSensorPosition(CLIMB6_POSITION_IN, 0, Constants.TALON_TIMEOUT_MS);

        m_climb10Motor.configFactoryDefault();
        m_climb10Motor.configSelectedFeedbackSensor( FeedbackDevice.CTRE_MagEncoder_Relative, 0, Constants.TALON_TIMEOUT_MS);
        m_climb10Motor.configNeutralDeadband(0.10, Constants.TALON_TIMEOUT_MS);
        // Configure Talon  SRX output and sensor direction
        m_climb10Motor.setSensorPhase(true);
        m_climb10Motor.setInverted(true);
        // Set peak current
        m_climb10Motor.configPeakCurrentLimit(35, Constants.TALON_TIMEOUT_MS);
        m_climb10Motor.configPeakCurrentDuration(200, Constants.TALON_TIMEOUT_MS);
        m_climb10Motor.configContinuousCurrentLimit(30, Constants.TALON_TIMEOUT_MS);
        m_climb10Motor.enableCurrentLimit(true);
        // Set Motion Magic gains in slot0
        m_climb10Motor.selectProfileSlot(0, 0);
        m_climb10Motor.config_kF(0, Constants.CLIMB6_MOTOR_KF, Constants.TALON_TIMEOUT_MS);
        m_climb10Motor.config_kP(0, kP.getDouble(Constants.CLIMB6_MOTOR_KP), Constants.TALON_TIMEOUT_MS);
        m_climb10Motor.config_kI(0, kI.getDouble(Constants.CLIMB6_MOTOR_KI), Constants.TALON_TIMEOUT_MS);
        m_climb10Motor.config_kD(0, kD.getDouble(Constants.CLIMB6_MOTOR_KD), Constants.TALON_TIMEOUT_MS);
        // Set acceleration and cruise velocity
        m_climb10Motor.configMotionCruiseVelocity(CLIMB6_MOTOR_CRUISE, Constants.TALON_TIMEOUT_MS);
        m_climb10Motor.configMotionAcceleration(CLIMB6_MOTOR_ACCELERATION, Constants.TALON_TIMEOUT_MS);
        //  Zero the sensor once on robot startup
        m_climb10Motor.setSelectedSensorPosition(CLIMB6_POSITION_IN, 0, Constants.TALON_TIMEOUT_MS);

        m_climb15Motor.configFactoryDefault();
        m_climb15Motor.configSelectedFeedbackSensor( FeedbackDevice.CTRE_MagEncoder_Relative, 0, Constants.TALON_TIMEOUT_MS);
        m_climb15Motor.configNeutralDeadband(0.10, Constants.TALON_TIMEOUT_MS);
        // Configure Talon  SRX output and sensor direction
        m_climb15Motor.setSensorPhase(false);
        // Set peak current
        m_climb15Motor.configPeakCurrentLimit(35, Constants.TALON_TIMEOUT_MS);
        m_climb15Motor.configPeakCurrentDuration(200, Constants.TALON_TIMEOUT_MS);
        m_climb15Motor.configContinuousCurrentLimit(30, Constants.TALON_TIMEOUT_MS);
        m_climb15Motor.enableCurrentLimit(true);
        // Set Motion Magic gains in slot0
        m_climb15Motor.selectProfileSlot(0, 0);
        m_climb15Motor.config_kF(0, Constants.CLIMB6_MOTOR_KF, Constants.TALON_TIMEOUT_MS);
        m_climb15Motor.config_kP(0, kP.getDouble(Constants.CLIMB6_MOTOR_KP), Constants.TALON_TIMEOUT_MS);
        m_climb15Motor.config_kI(0, kI.getDouble(Constants.CLIMB6_MOTOR_KI), Constants.TALON_TIMEOUT_MS);
        m_climb15Motor.config_kD(0, kD.getDouble(Constants.CLIMB6_MOTOR_KD), Constants.TALON_TIMEOUT_MS);
        // Set acceleration and cruise velocity
        m_climb15Motor.configMotionCruiseVelocity(3000, Constants.TALON_TIMEOUT_MS);
        m_climb15Motor.configMotionAcceleration(6000, Constants.TALON_TIMEOUT_MS);
        //  Zero the sensor once on robot startup
        m_climb15Motor.setSelectedSensorPosition(CLIMB6_POSITION_IN, 0, Constants.TALON_TIMEOUT_MS);
    }

    @Override
    public void periodic()
    {
        c15pos.setDouble( m_climb15Motor.getSelectedSensorPosition(0) );
        c15set.setDouble( c15Position );
        c10pos.setDouble( m_climb10Motor.getSelectedSensorPosition(0) );
        c10set.setDouble( c10Position );
        c6pos.setDouble( m_climb6Motor.getSelectedSensorPosition(0) );
        c6set.setDouble( c6Position );
    }

    public void setClimb6Speed(double value){
        c6Position = 0;
        if( value == 0.0 )
        {
            m_climb6Motor.configPeakCurrentLimit(35);
            m_climb6Motor.configContinuousCurrentLimit(30);
            m_climb6Motor.enableCurrentLimit(true);
        }
        else
        {
            m_climb6Motor.configPeakCurrentLimit(5);
            m_climb6Motor.configContinuousCurrentLimit(4);
            m_climb6Motor.enableCurrentLimit(true);
        }
        m_climb6Motor.set(ControlMode.PercentOutput, value);
    }

    public void setClimb10Speed(double value){
        c10Position = 0;
        if( value == 0.0 )
        {
            m_climb10Motor.configPeakCurrentLimit(35);
            m_climb10Motor.configContinuousCurrentLimit(30);
            m_climb10Motor.enableCurrentLimit(true);
        }
        else
        {
            m_climb10Motor.configPeakCurrentLimit(5);
            m_climb10Motor.configContinuousCurrentLimit(4);
            m_climb10Motor.enableCurrentLimit(true);
        }
        m_climb10Motor.set(ControlMode.PercentOutput, value);
    }

    public void setClimb15Speed(double value){
        c15Position = 0;
        if( value == 0.0 )
        {
            m_climb15Motor.configPeakCurrentLimit(35);
            m_climb15Motor.configContinuousCurrentLimit(30);
            m_climb15Motor.enableCurrentLimit(true);    
        }
        else
        {
            m_climb15Motor.configPeakCurrentLimit(5);
            m_climb15Motor.configContinuousCurrentLimit(4);
            m_climb15Motor.enableCurrentLimit(true);
        }
        m_climb15Motor.set(ControlMode.PercentOutput, value);
    }

    public void setClimb6Position(double value){
        c6Position = value;
        m_climb6Motor.set(ControlMode.MotionMagic, value);
    }
    public void setClimb10Position(double value){
        c10Position = value;
        m_climb10Motor.set(ControlMode.MotionMagic, value);
    }    

    public void setClimb15Position(double value){
        c15Position = value;
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
