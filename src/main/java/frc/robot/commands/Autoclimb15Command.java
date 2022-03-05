package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.ClimbSubsystem;

public class Autoclimb15Command extends SequentialCommandGroup {

    private ClimbSubsystem m_climbSubsystem;

    public Autoclimb15Command( ClimbSubsystem subsystem )
    {
      m_climbSubsystem = subsystem;
      addRequirements(m_climbSubsystem);

      // climb15 46882 fully extended
      // 51500 = climb10 fully extended
      addCommands( 
          sequence( 
            new Climb15PositionCommand( m_climbSubsystem, -5700 ),
            parallel(
              new Climb10PositionCommand( m_climbSubsystem, 0 ),
              new Climb15PositionCommand( m_climbSubsystem, 41300 )
            ),
            new Climb15PositionCommand( m_climbSubsystem, 47300 ),
            parallel(
              new Climb10PositionCommand( m_climbSubsystem, 51500 ),
              new Climb15PositionCommand( m_climbSubsystem, 21000 )
            ),
            new Climb15PositionCommand( m_climbSubsystem, 47300 )
          )
          .withTimeout( 10.0 )
        );

        /*
        Autoclimb 15 -  button 2 (while held): 
        0) (serial) retract Climb15 to 0" from full bottom (release 15 point hook from hook catc21
        1) (parallel) retract Climb10 to 0" from full bottom 
        2) (parallel) extend Climb15 to 21" from full down (end of travel)
        3) (serial) retract Climb15 to 19" from full down (seat hook15)
        4) (parallel) extend Climb10 to 21" from full down (disengage from the 10 point hook from bar)
        5) (parallel) retract Climb15 to 9" from full down 
        6) (serial) extend Climb15 to 21" from full down (hanging position)
        */
    }
    

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_climbSubsystem.setClimb6Speed(0);
    m_climbSubsystem.setClimb10Speed(0);
    m_climbSubsystem.setClimb15Speed(0);
  }
}
