package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.commands.KickerCommand;
import frc.robot.commands.ShooterWait;

public class KickerMultipleCommand extends SequentialCommandGroup {

  private ShooterSubsystem m_shooterSubsystem;
  private double m_output;

  public KickerMultipleCommand( ShooterSubsystem subsystem, double value )
  {
      m_shooterSubsystem = subsystem;
      m_output = value;

      addCommands(
          new KickerCommand( m_shooterSubsystem, m_output, true, false ).withTimeout(2.0),
          new KickerCommand( m_shooterSubsystem, -(m_output - 0.1), true, true ).withTimeout(1.0),
          new ShooterWait( m_shooterSubsystem ),
          new KickerCommand( m_shooterSubsystem, -(m_output - 0.1), true, false ).withTimeout(2.0),
          new WaitCommand( 0.2 ),
          new ShooterWait( m_shooterSubsystem ),
          new KickerCommand( m_shooterSubsystem, -(m_output - 0.1), true, false ).withTimeout(2.0)
      );
  
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_shooterSubsystem.setKickerSpeed(0.0);
  }
    
}
