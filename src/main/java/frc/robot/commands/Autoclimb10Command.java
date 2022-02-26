package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.ClimbSubsystem;

public class Autoclimb10Command extends SequentialCommandGroup {

    private ClimbSubsystem m_climbSubsystem;

    public Autoclimb10Command( ClimbSubsystem subsystem )
    {
      m_climbSubsystem = subsystem;
      addRequirements(m_climbSubsystem);

      addCommands( 
          new SequentialCommandGroup( 
            new ParallelCommandGroup(
              new Climb6PositionCommand( m_climbSubsystem, 0 ),
              new SequentialCommandGroup(
                new Climb10PositionCommand( m_climbSubsystem, -2000 ),
                new Climb10PositionCommand( m_climbSubsystem, 50000 )
              )
            ),
            new Climb10PositionCommand(m_climbSubsystem, 47500 ),
            new Climb6PositionCommand(m_climbSubsystem, 20000 ),
            new Climb10PositionCommand(m_climbSubsystem, 12500 )
          )
          .withTimeout( 10.0 )
        );

        /*
        Autoclimb 10 - button 1 (while held): 
        1) (parallel) retract Climb6 to 0" from bottom
        2) (parallel) retract Climb10 1" to release hook10 from catch + extend Climb10 to 21" from full down (end of travel)
        3) (serial) retract Climb10 to 20" from full down (seat 10 hook)
        4) (serial) extend Climb6 to 8" from full down (disengage from the 6 point hook from bar)
        5) (serial) retract Climb10 to 21" from full down.  Note: we really only need to go 9" from full down if transitioning into 15 point climb
        */

    }
    

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
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
    m_climbSubsystem.setClimb6Speed(0);
    m_climbSubsystem.setClimb10Speed(0);
    m_climbSubsystem.setClimb15Speed(0);
    super.end(interrupted);
  }
}
