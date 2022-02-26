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

      addCommands( 
          sequence( 
            new Climb15PositionCommand( m_climbSubsystem, -2000 ),
            parallel(
              new Climb10PositionCommand( m_climbSubsystem, 0 ),
              new Climb15PositionCommand( m_climbSubsystem, 50000 )
            ),
            new Climb15PositionCommand( m_climbSubsystem, 45000 ),
            parallel(
              new Climb10PositionCommand( m_climbSubsystem, 52500 ),
              new Climb15PositionCommand( m_climbSubsystem, 22500 )
            ),
            new Climb15PositionCommand( m_climbSubsystem, 52500 )
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
      return false;
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
