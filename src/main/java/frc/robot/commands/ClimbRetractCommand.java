package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ClimbSubsystem;

public class ClimbRetractCommand  extends CommandBase {

    private ClimbSubsystem m_climbSubsystem;

    public ClimbRetractCommand( ClimbSubsystem subsystem )
    {
        m_climbSubsystem = subsystem;

        addRequirements(m_climbSubsystem);
    }
    
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    super.initialize();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_climbSubsystem.retractClimb();
    super.execute();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
      return false;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_climbSubsystem.stopClimb();
    super.end(interrupted);
  }
}
