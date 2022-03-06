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
          new ShooterWait( m_shooterSubsystem ),
          new KickerCommand( m_shooterSubsystem, m_output, true ),
          new WaitCommand( 0.2 ),
          new KickerCommand( m_shooterSubsystem, 0.0, true ),
          new WaitCommand( 0.5 ),
          new ShooterWait( m_shooterSubsystem ),
          new KickerCommand( m_shooterSubsystem, m_output, true ),
          new WaitCommand( 2.0 )
      );
  
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_shooterSubsystem.setKickerSpeed(0.0);
  }
    
}
