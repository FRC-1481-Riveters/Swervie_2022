package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.Constants;

public class IntakeRetractCommandOld extends SequentialCommandGroup {

  private IntakeSubsystem m_intakeSubsystem;
  private ShooterSubsystem m_shooterSubsystem;

  public IntakeRetractCommandOld( IntakeSubsystem subsystem, ShooterSubsystem shooterSubsystem )
  {
    m_intakeSubsystem = subsystem;
    m_shooterSubsystem = shooterSubsystem;

    addRequirements(m_intakeSubsystem);

    addCommands
    (
        new IntakePositionCommand( m_intakeSubsystem, Constants.INTAKE_ARM_POSITION_IN_FULL ),
        // run the kicker forward slowly until the light curtain is blocked
        new KickerCommand( m_shooterSubsystem, 0.3, true, true, 0 ),
        new WaitCommand(1.0)
    );

  }

  @Override
  public void end(boolean interrupted) {
  }

}
