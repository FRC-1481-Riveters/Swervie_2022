package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.Constants;

public class IntakeRetractCommand extends SequentialCommandGroup {

  private IntakeSubsystem m_intakeSubsystem;

  public IntakeRetractCommand( IntakeSubsystem subsystem )
  {
    m_intakeSubsystem = subsystem;

    addRequirements(m_intakeSubsystem);

    addCommands(
        new IntakePositionCommand( m_intakeSubsystem, Constants.INTAKE_ARM_POSITION_IN ),
        new WaitCommand( 2.0 ),
        new IntakePositionCommand( m_intakeSubsystem, Constants.INTAKE_ARM_POSITION_IN_FULL )
    );

  }

  @Override
  public void end(boolean interrupted) {
    super.end( interrupted );
  }

}
