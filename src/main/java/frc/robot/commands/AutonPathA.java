package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.ClimbSubsystem;

public class AutonPathA extends SequentialCommandGroup {

    private DrivetrainSubsystem m_drivetrainSubsystem;
    private IntakeSubsystem m_intakeSubsystem;
    private ShooterSubsystem m_shooterSubsystem;
    private String m_autonPath;
    private ClimbSubsystem m_climbSubsystem;

    public AutonPathA( RobotContainer container, String autonpath )
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
        parallel(
          new AutonMacroPlayback( m_autonPath, m_drivetrainSubsystem ),
          sequence(
            parallel(
              new IntakePositionCommand(m_intakeSubsystem, Constants.INTAKE_ARM_POSITION_OUT),
              new IntakeArmRollerCommand(m_intakeSubsystem, 0.75)
              .withTimeout(3.0)
            ),
            new IntakePositionCommand(m_intakeSubsystem, Constants.INTAKE_ARM_POSITION_IN),
            //TODO !*!*!* need a path to turn around!
            new AutonMacroPlayback( "/home/lvuser/turn180.csv", m_drivetrainSubsystem ),
            parallel(
              new AutoAimCommand( m_drivetrainSubsystem),
              new AutoDriveCommand( m_drivetrainSubsystem, 0.0, 0.0, 0.0 )
            ).withTimeout(2.0),
            parallel(
              new ShooterYeetCommandPart2ElectricBoogaloo(m_shooterSubsystem, Constants.YEET_SPEED_HIGH),
              sequence(
                new KickerMultipleCommand( m_shooterSubsystem, 0.7, m_intakeSubsystem )
              )
            )
            .withTimeout(5.0)
          )
          .withTimeout(15.0)
        )
      );

    }
}
