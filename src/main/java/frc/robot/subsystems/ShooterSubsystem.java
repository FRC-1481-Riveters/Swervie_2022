package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import static frc.robot.Constants.*;

public class ShooterSubsystem extends SubsystemBase{
    private CANSparkMax m_yeetMotor = new CANSparkMax(YEET_MOTOR, MotorType.kBrushless);
    TalonSRX m_kickerMotor = new TalonSRX(KICKER_MOTOR);

    public void setYeetSpeed (double value){
        m_yeetMotor.set(value);
    }

    public void setKickerSpeed(double value){
        m_kickerMotor.set(ControlMode.PercentOutput, value);
    }
}
