package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.ClimbSubsystem;
import frc.robot.subsystems.DrivetrainSubsystem;

public class Autoclimb10Command extends SequentialCommandGroup {

    private ClimbSubsystem m_climbSubsystem;
    private DrivetrainSubsystem m_driveSubsystem;

    public Autoclimb10Command( ClimbSubsystem subsystem, DrivetrainSubsystem drivetrainSubsystem )
    {
      m_climbSubsystem = subsystem;
      m_driveSubsystem = drivetrainSubsystem;

      // 48500 = climb10 fully extended
      addCommands( 
          sequence( 
            parallel(
              sequence(
                new Climb6PositionCommand( m_climbSubsystem, 30000 ),
                new WaitCommand(0.2),
                new Climb6PositionCommand( m_climbSubsystem, 27000 ),
                new WaitCommand(0.2),
                new Climb6PositionCommand( m_climbSubsystem, 23000 ),
                new WaitCommand(0.2),
                new Climb6PositionCommand( m_climbSubsystem, 12000 )
              ),
              sequence(
                new Climb10PositionCommand( m_climbSubsystem, -4000 ),  // unlatch climb10
                new Climb10PositionCommand( m_climbSubsystem, 50500 )   // 10 extend full
              )
            ),
            new Climb6PositionCommand( m_climbSubsystem, -3400 ), // 6 retract full
            new WaitCommand(1.0),
            new Climb10PositionCommand(m_climbSubsystem, 44500 ), // 10 cinched
            new WaitCommand(0.05),
            new Climb6PositionCommand(m_climbSubsystem, 28000 ),
            parallel(
              new Climb10PositionCommand(m_climbSubsystem, 22500 ),
              new Climb6PositionCommand(m_climbSubsystem, 51400 )
            ),
            new Autoclimb15Command( m_climbSubsystem, m_driveSubsystem )
          )
          .withTimeout( 20.0 )
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

  @Override
  public void initialize()
  {
     System.out.println("Autoclimb10Command initialize");
     super.initialize();
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
