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
  private double m_transition;
  private boolean m_direction;  // FALSE for negative (towards 6), TRUE for positive (toward 15)

  public GyroRollTransitionCommand( DrivetrainSubsystem driveSubsystem, double transition, boolean direction )
  {
    m_previousRoll = 0;
    m_driveSubsystem = driveSubsystem;
    m_rollFilter = new MedianFilter(5);
    m_transition = transition;
    m_direction = direction;
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    boolean bFinished = false;
    double gyroRoll;
    gyroRoll = m_driveSubsystem.gyroGetRoll();
    gyroRoll = m_rollFilter.calculate(gyroRoll);
    if( m_direction == true )
    {
      if( (m_previousRoll <= m_transition) && (gyroRoll > m_transition) )
        bFinished = true;
    }
    else
    {
      if( (m_previousRoll >= m_transition) && (gyroRoll < m_transition) )
        bFinished = true;
    }
    m_previousRoll = gyroRoll;
    return bFinished;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

}
