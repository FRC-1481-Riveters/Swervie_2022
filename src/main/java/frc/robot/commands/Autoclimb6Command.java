package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.ClimbSubsystem;

public class Autoclimb6Command extends SequentialCommandGroup {

    private ClimbSubsystem m_climbSubsystem;
    public Autoclimb6Command( ClimbSubsystem subsystem )
    {
      m_climbSubsystem = subsystem;
      addRequirements(m_climbSubsystem);

      addCommands(
        new Climb6PositionCommand(m_climbSubsystem, 8000)
        .withTimeout(5.0)
      );

      /*addCommands( 
          sequence( 
            parallel(
              new Climb6PositionCommand( m_climbSubsystem, 12000 ),
              new Climb10PositionCommand( m_climbSubsystem, -2500 ),
              new Climb10PositionCommand( m_climbSubsystem, 53300 )
            ),
            new Climb6PositionCommand( m_climbSubsystem, 0 ),
            new Climb10PositionCommand(m_climbSubsystem, 50000 ),
            new Climb6PositionCommand(m_climbSubsystem, 15672 ),
            new Climb10PositionCommand(m_climbSubsystem, 22500 )
          )
          .withTimeout( 10.0 )
        );*/

        /*
        Autoclimb 10 - button 1 (while held): 
        1) (parallel) retract Climb6 to 0" from bottom
        2) (parallel) retract Climb10 1" to release hook10 from catch + extend Climb10 to 21" from full down (end of travel)
        3) (serial) retract Climb10 to 20" from full down (seat 10 hook)
        4) (serial) extend Climb6 to 8" from full down (disengage from the 6 point hook from bar)
        5) (serial) retract Climb10 to 21" from full down.  Note: we really only need to go 9" from full down if transitioning into 15 point climb
        */
    }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_climbSubsystem.setClimb6Speed(0);
    m_climbSubsystem.setClimb10Speed(0);
  }
}
