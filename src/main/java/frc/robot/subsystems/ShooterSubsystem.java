package frc.robot.subsystems;

import edu.wpi.first.networktables.EntryListenerFlags;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.*;

import com.revrobotics.CANSparkMax;
import com.revrobotics.*;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.SparkMaxRelativeEncoder;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import static frc.robot.Constants.*;

public class ShooterSubsystem extends SubsystemBase{
    private double m_shooterIntendedSpeed = 10000.0;
    private CANSparkMax m_yeetMotor = new CANSparkMax(YEET_MOTOR, MotorType.kBrushless);
    private SparkMaxRelativeEncoder m_encoder = (SparkMaxRelativeEncoder) m_yeetMotor.getEncoder();
    TalonSRX m_kickerMotor = new TalonSRX(KICKER_MOTOR);
    private SparkMaxPIDController m_pidController = m_yeetMotor.getPIDController();
    private NetworkTableEntry shooterKp;
    private NetworkTableEntry shooterKi;
    private NetworkTableEntry shooterKd;
    private NetworkTableEntry shooterKf;
    
    public ShooterSubsystem(){
      //m_yeetMotor.restoreFactoryDefaults();
      m_yeetMotor.setInverted(true);
      m_pidController.setP(0.0001);
      m_pidController.setI(0.000000005);
      m_pidController.setD(0.0001);
      m_pidController.setFF(0.00018);
      setYeetSpeed(0.0);

      shooterKp = NetworkTableInstance.getDefault().getTable("SmartDashboard").getEntry("Shooter kP");
      shooterKi = NetworkTableInstance.getDefault().getTable("SmartDashboard").getEntry("Shooter kI");
      shooterKd = NetworkTableInstance.getDefault().getTable("SmartDashboard").getEntry("Shooter kD");
      shooterKf = NetworkTableInstance.getDefault().getTable("SmartDashboard").getEntry("Shooter kF");

      shooterKp.setDouble(m_pidController.getP());
      shooterKi.setDouble(m_pidController.getI());
      shooterKd.setDouble(m_pidController.getD());
      shooterKf.setDouble(m_pidController.getFF());

    }

    public void setYeetSpeed (double RPM){
        m_yeetMotor.set(RPM);
        m_shooterIntendedSpeed = RPM;
        if (m_shooterIntendedSpeed > 10.0) {
            m_pidController.setReference(RPM, ControlType.kVelocity);
          } else {
            m_yeetMotor.set(0.0);
          }
        
    }

    public void setKickerSpeed(double value){
        m_kickerMotor.set(ControlMode.PercentOutput, value);
    }
    public double getSpeed() {
        return m_encoder.getVelocity();
      }
    
      public boolean isAtSpeed() {
        if (Math.abs(
            (getSpeed() - m_shooterIntendedSpeed) / m_shooterIntendedSpeed) <= YEET_SPEED_TOLERANCE) {
          return true;
        } else {
          return false;
        }
      }
    
}
