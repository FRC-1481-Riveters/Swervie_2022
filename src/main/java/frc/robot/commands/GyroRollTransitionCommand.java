package frc.robot.commands;

import edu.wpi.first.math.filter.MedianFilter;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.math.filter.MedianFilter;
import frc.robot.subsystems.DrivetrainSubsystem;
import edu.wpi.first.networktables.NetworkTableInstance;

public class GyroRollTransitionCommand extends CommandBase {

  double m_previousRoll;
  private MedianFilter m_rollFilter;
  private DrivetrainSubsystem m_driveSubsystem;

  public GyroRollTransitionCommand( DrivetrainSubsystem driveSubsystem )
  {
    m_previousRoll = 0;
    m_driveSubsystem = driveSubsystem;
    m_rollFilter = new MedianFilter(5);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    double gyroRoll;
    gyroRoll = m_driveSubsystem.gyroGetRoll();
    gyroRoll = m_rollFilter.calculate(gyroRoll);
    if( (m_previousRoll < -85.0) && (gyroRoll > -85.0) )
        return true;
    else
    {
        m_previousRoll = gyroRoll;
        return false;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    super.end( interrupted );
  }

}
