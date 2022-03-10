package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class AutonPathA extends SequentialCommandGroup {

    private DrivetrainSubsystem m_drivetrainSubsystem;
    private IntakeSubsystem m_intakeSubsystem;
    private ShooterSubsystem m_shooterSubsystem;
    private String m_autonPath;

    public AutonPathA( RobotContainer container, String autonpath )
    {
      m_drivetrainSubsystem = container.getDrivetrainSubsystem();
      m_intakeSubsystem = container.getIntakeSubsystem();
      m_shooterSubsystem = container.getShooterSubsystem();
      m_autonPath = autonpath;

      addCommands(
        parallel(
          new AutonMacroPlayback( m_autonPath, m_drivetrainSubsystem ),
          sequence(
            parallel(
              new IntakePositionCommand(m_intakeSubsystem, Constants.INTAKE_ARM_POSITION_OUT),
              new IntakeArmRollerCommand(m_intakeSubsystem, 0.75)
              .withTimeout(5.0)
            ),
            new IntakePositionCommand(m_intakeSubsystem, Constants.INTAKE_ARM_POSITION_IN),
            new WaitCommand(3.5),
            parallel(
              new ShooterYeetCommandPart2ElectricBoogaloo(m_shooterSubsystem, 3800),
              sequence(
                new WaitCommand(2.0),
                new KickerMultipleCommand( m_shooterSubsystem, -0.5 )
              )
            )
            .withTimeout(5.0)
          )
          .withTimeout(15.0)
        )
      );

    }
}
