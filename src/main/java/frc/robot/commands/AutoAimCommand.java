package frc.robot.commands;

import edu.wpi.first.math.filter.MedianFilter;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DrivetrainSubsystem;
import edu.wpi.first.networktables.NetworkTableInstance;

public class AutoAimCommand extends CommandBase {

    private MedianFilter m_angleFilter;
    private DrivetrainSubsystem m_drivetrainSubsystem;

    public AutoAimCommand( DrivetrainSubsystem drivetrainSubsystem )
    {
        m_drivetrainSubsystem = drivetrainSubsystem;
        m_angleFilter = new MedianFilter(5);
    }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double angle;
    angle = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx").getDouble(0); // get target angle
    angle = angle * -0.1;
    if( Math.abs(angle) < 0.08 ) angle = 0;
    angle = m_angleFilter.calculate(angle);
    m_drivetrainSubsystem.autoAimAngle = angle * 0.3;
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_drivetrainSubsystem.autoAimAngle = 0;
  }

}
