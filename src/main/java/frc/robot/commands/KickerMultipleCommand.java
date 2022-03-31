package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.Constants;
import frc.robot.subsystems.IntakeSubsystem;

public class KickerMultipleCommand extends SequentialCommandGroup {

  private ShooterSubsystem m_shooterSubsystem;
  private double m_output;
  private IntakeSubsystem m_intakeSubsystem;

  public KickerMultipleCommand( ShooterSubsystem subsystem, double value, IntakeSubsystem intake )
  {
      m_shooterSubsystem = subsystem;
      m_output = value;
      m_intakeSubsystem = intake;

      addCommands(
          // run the kicker backwards until the light curtain is not blocked
          new KickerCommand( m_shooterSubsystem, -m_output, true, false, 0 ).withTimeout(0.5),

          // run the kicker forward slowly until the light curtain is blocked
          new KickerCommand( m_shooterSubsystem, 0.3, true, true, 0),

          // wait for the shooter wheel to be at the right speed
          new ShooterWait( m_shooterSubsystem ),

          // run the kicker forwards until the light curtain is not blocked (ball is gone)
          new KickerCommand( m_shooterSubsystem, m_output, true, false, 200 ).withTimeout(2.0),

          //bring intake in
          new IntakePositionCommand( m_intakeSubsystem, Constants.INTAKE_ARM_POSITION_IN_FULL ),

          // run the kicker forward slowly until the light curtain is blocked
          new KickerCommand( m_shooterSubsystem, 0.3, true, true, 200 ),

          // wait for the shooter wheel to be at the right speed
          new ShooterWait( m_shooterSubsystem ),
          new WaitCommand(0.1),

          // run the kicker forwards until the light curtain is not blocked (ball is gone)
          new KickerCommand( m_shooterSubsystem, m_output, true, false, 200 ).withTimeout(2.0)
      );
  
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    super.initialize();
    System.out.format( " %.3f KickerMultipleCommand initialize%n", Timer.getMatchTime() );
  }
  
  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    System.out.format( " %.3f KickerMultipleCommand end%n", Timer.getMatchTime() );
    m_shooterSubsystem.setKickerSpeed(0.0);
  }
    
}
