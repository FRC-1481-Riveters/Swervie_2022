package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.networktables.NetworkTableEntry;

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

    private ShuffleboardTab tab;
    private NetworkTableEntry kP, kI, kD, kCruise, kAcceleration;
    private NetworkTableEntry widgetClimb6Position, widgetClimb6Setpoint, widgetClimb6Current, widgetClimb6Output;
    private NetworkTableEntry widgetClimb10Position, widgetClimb10Setpoint, widgetClimb10Current, widgetClimb10Output;
    private NetworkTableEntry widgetClimb15Position, widgetClimb15Setpoint, widgetClimb15Current, widgetClimb15Output;
    private NetworkTableEntry widgetBatteryVoltage;


    public void ClimbSubsystemInit() {

        tab = Shuffleboard.getTab("Climb");
        kP = tab.add("kP",CLIMB6_MOTOR_KP).getEntry();
        kI = tab.add("kI",CLIMB6_MOTOR_KI).getEntry();
        kD = tab.add("kD",CLIMB6_MOTOR_KD).getEntry();
        kCruise = tab.add("kCruise",CLIMB6_MOTOR_CRUISE).getEntry();
        kAcceleration = tab.add("kAcceleration",CLIMB6_MOTOR_ACCELERATION).getEntry();
        widgetClimb6Position  = tab.add("C6Pos",0).withWidget("Graph").getEntry();
        widgetClimb6Setpoint  = tab.add("C6Set",0).withWidget("Graph").getEntry();
        widgetClimb6Current   = tab.add("C6Cur",0).withWidget("Graph").getEntry();
        widgetClimb6Output    = tab.add("C6Out",0).withWidget("Graph").getEntry();
        widgetClimb10Position = tab.add("C10Pos",0).withWidget("Graph").getEntry();
        widgetClimb10Setpoint = tab.add("C10Set",0).withWidget("Graph").getEntry();
        widgetClimb10Current  = tab.add("C10Cur",0).withWidget("Graph").getEntry();
        widgetClimb10Output   = tab.add("C10Out",0).withWidget("Graph").getEntry();
        widgetClimb15Position = tab.add("C15Pos",0).withWidget("Graph").getEntry();
        widgetClimb15Setpoint = tab.add("C15Set",0).withWidget("Graph").getEntry();
        widgetClimb15Current  = tab.add("C15Cur",0).withWidget("Graph").getEntry();
        widgetClimb15Output   = tab.add("C15Out",0).withWidget("Graph").getEntry();
        widgetBatteryVoltage = tab.add("Battery",0).withWidget("Graph").getEntry();
    
        m_climb6Motor.configFactoryDefault();
        m_climb6Motor.configSelectedFeedbackSensor( FeedbackDevice.CTRE_MagEncoder_Relative, 0, Constants.TALON_TIMEOUT_MS);
        m_climb6Motor.configNeutralDeadband(0.10, Constants.TALON_TIMEOUT_MS);
        // Configure Talon  SRX output and sensor direction
        m_climb6Motor.setSensorPhase(true);
        m_climb6Motor.setInverted(true);
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
        m_climb15Motor.setInverted(false);
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
        m_climb15Motor.config_kP(0, kP.getDouble(Constants.CLIMB6_MOTOR_KP), Constants.TALON_TIMEOUT_MS);
        m_climb15Motor.config_kI(0, kI.getDouble(Constants.CLIMB6_MOTOR_KI), Constants.TALON_TIMEOUT_MS);
        m_climb15Motor.config_kD(0, kD.getDouble(Constants.CLIMB6_MOTOR_KD), Constants.TALON_TIMEOUT_MS);
        // Set acceleration and cruise velocity
        m_climb15Motor.configMotionCruiseVelocity(CLIMB6_MOTOR_CRUISE, Constants.TALON_TIMEOUT_MS);
        m_climb15Motor.configMotionAcceleration(CLIMB6_MOTOR_ACCELERATION, Constants.TALON_TIMEOUT_MS);
        //  Zero the sensor once on robot startup
        m_climb15Motor.setSelectedSensorPosition(CLIMB6_POSITION_IN, 0, Constants.TALON_TIMEOUT_MS);
    }
    
    @Override
    public void periodic() {
      // This method will be called once per scheduler run
      widgetClimb6Position.setNumber( m_climb6Motor.getSelectedSensorPosition(0));
      widgetClimb6Setpoint.setNumber( m_climb6Motor.getClosedLoopTarget());
      widgetClimb6Current .setNumber( m_climb6Motor.getSupplyCurrent());
      widgetClimb6Output  .setNumber( m_climb6Motor.getMotorOutputVoltage());
      widgetClimb10Position.setNumber( m_climb10Motor.getSelectedSensorPosition(0));
      widgetClimb10Setpoint.setNumber( m_climb10Motor.getClosedLoopTarget());
      widgetClimb10Current .setNumber( m_climb10Motor.getSupplyCurrent());
      widgetClimb10Output  .setNumber( m_climb10Motor.getMotorOutputVoltage());
      widgetClimb15Position.setNumber( m_climb15Motor.getSelectedSensorPosition(0));
      widgetClimb15Setpoint.setNumber( m_climb15Motor.getClosedLoopTarget());
      widgetClimb15Current .setNumber( m_climb15Motor.getSupplyCurrent());
      widgetClimb15Output  .setNumber( m_climb15Motor.getMotorOutputVoltage());
      widgetBatteryVoltage.setNumber( RobotController.getBatteryVoltage() );
    }

    public void setClimb6Speed(double value){
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
        m_climb6Motor.set(ControlMode.MotionMagic, value);
    }
    public void setClimb10Position(double value){
        m_climb10Motor.set(ControlMode.MotionMagic, value);
    }    

    public void setClimb15Position(double value){
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
