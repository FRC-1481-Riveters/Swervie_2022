package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

public class AutonPathA extends SequentialCommandGroup {

    private DrivetrainSubsystem m_drivetrainSubsystem;
    private IntakeSubsystem m_intakeSubsystem;
    private String m_autonPath;

    public AutonPathA( RobotContainer container, String autonpath )
    {
      m_drivetrainSubsystem = container.getDrivetrainSubsystem();
      m_intakeSubsystem = container.getIntakeSubsystem();
      m_autonPath = autonpath;

      addCommands(
        parallel(
          new AutonMacroPlayback( m_autonPath, m_drivetrainSubsystem ),
          new IntakePositionCommand( m_intakeSubsystem, Constants.INTAKE_ARM_POSITION_IN_FULL )
          .withTimeout(5.0)
        )
      );

    }
}
