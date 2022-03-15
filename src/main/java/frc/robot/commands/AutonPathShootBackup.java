package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.ClimbSubsystem;

public class AutonPathShootBackup extends SequentialCommandGroup {

    private DrivetrainSubsystem m_drivetrainSubsystem;
    private IntakeSubsystem m_intakeSubsystem;
    private ShooterSubsystem m_shooterSubsystem;
    private String m_autonPath;
    private ClimbSubsystem m_climbSubsystem;

    public AutonPathShootBackup( RobotContainer container, String autonpath )
    {
      m_drivetrainSubsystem = container.getDrivetrainSubsystem();
      m_intakeSubsystem = container.getIntakeSubsystem();
      m_shooterSubsystem = container.getShooterSubsystem();
      m_autonPath = autonpath;
      m_climbSubsystem = container.getClimbSubsystem();

      addCommands(
        parallel(
          new Climb6PositionCommand(m_climbSubsystem, 600),
          new Climb10PositionCommand(m_climbSubsystem, 600),
          new Climb15PositionCommand(m_climbSubsystem, 600)
        ),
        sequence(
          parallel(
            new ShooterYeetCommandPart2ElectricBoogaloo(m_shooterSubsystem, 3800),
              sequence(
                new WaitCommand(2.0),
                new KickerMultipleCommand( m_shooterSubsystem, -0.5 )
            )
          )
          .withTimeout(5.0),
          new AutonMacroPlayback( m_autonPath, m_drivetrainSubsystem )
          )
          .withTimeout(15.0)
        );
    }
}
