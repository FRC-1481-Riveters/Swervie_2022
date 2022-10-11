package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ClimbSubsystem;

public class Climb15PositionCommand extends CommandBase {

    private ClimbSubsystem m_climbSubsystem;
    private double m_setPosition;

    public Climb15PositionCommand( ClimbSubsystem subsystem, double position )
    {
        m_climbSubsystem = subsystem;
        m_setPosition = position;
    }
    
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_climbSubsystem.setClimb15Position(m_setPosition);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
      if( Math.abs(m_setPosition - m_climbSubsystem.getClimb15Position()) < 300 )
      {
        return true;
      }
      else
      {
        return false;
      }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }
}
