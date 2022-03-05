package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.ClimbSubsystem;

public class Autoclimb6StartCommand extends SequentialCommandGroup {

    private ClimbSubsystem m_climbSubsystem;
    public Autoclimb6StartCommand( ClimbSubsystem subsystem )
    {
      m_climbSubsystem = subsystem;
      addRequirements(m_climbSubsystem);

      addCommands(
        // 51000 = full travel
        new Climb6PositionCommand(m_climbSubsystem, -3800)
        .withTimeout(2.0),
        new Climb6PositionCommand(m_climbSubsystem, 51000)
        .withTimeout(5.0)
      );

    }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_climbSubsystem.setClimb6Speed(0);
    m_climbSubsystem.setClimb10Speed(0);
  }
}
