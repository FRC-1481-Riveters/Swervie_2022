package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ClimbSubsystem;

public class Climb10ManualCommand extends CommandBase {

    private ClimbSubsystem m_climbSubsystem;
    private double m_output;

    public Climb10ManualCommand( ClimbSubsystem subsystem, double value )
    {
        m_climbSubsystem = subsystem;
        m_output = value;

        addRequirements(m_climbSubsystem);
    }
    
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_climbSubsystem.setClimb10Speed(m_output);
    super.initialize();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    super.execute();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
      return true;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_climbSubsystem.setClimb10Speed(0);
    super.end(interrupted);
  }
}
