package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ClimbSubsystem;

public class Climb6PositionCommand extends CommandBase {

    private ClimbSubsystem m_climbSubsystem;
    private double m_setPosition;

    public Climb6PositionCommand( ClimbSubsystem subsystem, double position )
    {
        m_climbSubsystem = subsystem;
        m_setPosition = position;
    }
    
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_climbSubsystem.setClimb6Position(m_setPosition);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
      if( Math.abs(m_setPosition - m_climbSubsystem.getClimb6Position()) < 300 )
      {
        return true;
      }
      else
      {
        return false;
      }
  }
}
