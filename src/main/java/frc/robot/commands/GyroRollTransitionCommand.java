package frc.robot.commands;

import edu.wpi.first.math.filter.MedianFilter;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DrivetrainSubsystem;
import edu.wpi.first.networktables.NetworkTableInstance;

public class GyroRollTransitionCommand extends CommandBase {

  double m_previousRoll;
  private DrivetrainSubsystem m_driveSubsystem;

  public GyroRollTransitionCommand( DrivetrainSubsystem driveSubsystem )
  {
    m_previousRoll = 0;
    m_driveSubsystem = driveSubsystem;
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    double gyroRoll;
    gyroRoll = m_driveSubsystem.gyroGetRoll();
    if( (m_previousRoll > 0) && (gyroRoll < 0) )
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
