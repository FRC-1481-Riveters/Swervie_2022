package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.ClimbSubsystem;
import frc.robot.subsystems.DrivetrainSubsystem;

public class Autoclimb15Command extends SequentialCommandGroup {

    private ClimbSubsystem m_climbSubsystem;
    private DrivetrainSubsystem m_driveSubsystem;

    public Autoclimb15Command( ClimbSubsystem subsystem, DrivetrainSubsystem drivetrainSubsystem )
    {
      m_driveSubsystem = drivetrainSubsystem;
      m_climbSubsystem = subsystem;
      addRequirements(m_climbSubsystem);

      // climb15 45700 fully extended (not including overtravel slop)
      // 50500 = climb10 fully extended (including overtravel slop)
      addCommands( 
          sequence( 
            parallel(
              new Climb6PositionCommand( m_climbSubsystem, 51400 ), 
              new Climb15PositionCommand( m_climbSubsystem, -4400 ),
              new WaitCommand(0.2)
            ),
            parallel(
              new Climb10PositionCommand( m_climbSubsystem, -4000 ),
              new Climb15PositionCommand( m_climbSubsystem, 24500 )
            ),
            new GyroRollTransitionCommand( m_driveSubsystem, 10.0, true ),
            new Climb15PositionCommand( m_climbSubsystem, 45700 ),
            new GyroRollTransitionCommand( m_driveSubsystem, 0.0, false ),
            parallel(
              new Climb6PositionCommand( m_climbSubsystem, -3400 ), 
              new Climb10PositionCommand( m_climbSubsystem, 50500 )
            ),
            new Climb15PositionCommand( m_climbSubsystem, 47300 )
            //!*!*!* TODO: MAKE JOYSTICK RUMBLE WHEN DONE
          )
          .withTimeout( 15.0 )
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
