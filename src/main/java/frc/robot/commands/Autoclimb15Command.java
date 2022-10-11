package frc.robot.commands;

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

      // climb15 45700 fully extended (not including overtravel slop)
      // 50500 = climb10 fully extended (including overtravel slop)
      addCommands( 
          sequence( 
            parallel(
              // Fully extend climb6
              new Climb6PositionCommand( m_climbSubsystem, 51400 ), 
              // Unlatch Climb15
              new Climb15PositionCommand( m_climbSubsystem, -4400 ),
              // Small delay to allow Climb15 hooks to swing out
              new WaitCommand(0.2)
            ),
            parallel(
              // Fully retract Climb10
              new Climb10PositionCommand( m_climbSubsystem, -4000 ),
              // Move Climb15 just short of 15 point bar
              new Climb15PositionCommand( m_climbSubsystem, 22500 )
            ),
            // Wait until robot is swinging up towards 15 point bar
//            new GyroRollTransitionCommand( m_driveSubsystem, 12.0, true ),
            // Fully extend Climb15
            new Climb15PositionCommand( m_climbSubsystem, 45700 ),
            // Wait until robot is swinging down towards 6 point bar
//            new GyroRollTransitionCommand( m_driveSubsystem, 2.0, false ),
            parallel(
              // Fully retract Climb6 (just to keep it out of the way)
              new Climb6PositionCommand( m_climbSubsystem, -3400 ), 
              // Fully extend Climb10 (settle Climb15 hooks on 15 point bar)
              new Climb10PositionCommand( m_climbSubsystem, 50500 )
            ),
            // Fully extend Climb15 (swing onto 15 point bar)
            new Climb15PositionCommand( m_climbSubsystem, 47300 )
          )
          .withTimeout( 15.0 )
        );
    }
    
  @Override
  public void initialize()
  {
      System.out.println("Autoclimb15Command initialize");
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
