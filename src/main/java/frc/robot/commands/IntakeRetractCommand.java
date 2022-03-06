package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.Constants;
import frc.robot.commands.IntakePositionCommand;

public class IntakeRetractCommand extends SequentialCommandGroup {

  private IntakeSubsystem m_intakeSubsystem;

  public IntakeRetractCommand( IntakeSubsystem subsystem )
  {
    addCommands(
        new IntakePositionCommand( m_intakeSubsystem, Constants.INTAKE_ARM_POSITION_IN ),
        new WaitCommand( 2.0 ),
        new IntakePositionCommand( m_intakeSubsystem, Constants.INTAKE_ARM_POSITION_IN_FULL )
    );

    m_intakeSubsystem = subsystem;

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }

}
