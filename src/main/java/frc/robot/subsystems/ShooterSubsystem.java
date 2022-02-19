package frc.robot.subsystems;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.SparkMaxRelativeEncoder;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import static frc.robot.Constants.*;

public class ShooterSubsystem extends SubsystemBase{
    private double m_shooterIntendedSpeed = 10000.0;
    private SparkMaxPIDController m_pidController;
    private CANSparkMax m_yeetMotor = new CANSparkMax(YEET_MOTOR, MotorType.kBrushless);
    private SparkMaxRelativeEncoder m_encoder = (SparkMaxRelativeEncoder) m_yeetMotor.getEncoder();
    TalonSRX m_kickerMotor = new TalonSRX(KICKER_MOTOR);

    public void setYeetSpeed (double value){
        m_yeetMotor.set(value);
      //  if (m_shooterIntendedSpeed > 10.0) {
      //      m_pidController.setReference(value, ControlType.kVelocity);
      //    } else {
      //      m_yeetMotor.set(0.0);
      //    }
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
