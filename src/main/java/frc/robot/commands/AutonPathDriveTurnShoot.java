package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.ClimbSubsystem;

public class AutonPathDriveTurnShoot extends SequentialCommandGroup {

    private DrivetrainSubsystem m_drivetrainSubsystem;
    private IntakeSubsystem m_intakeSubsystem;
    private ShooterSubsystem m_shooterSubsystem;
    private String m_autonPath;
    private ClimbSubsystem m_climbSubsystem;

    public AutonPathDriveTurnShoot( RobotContainer container, String autonpath )
    {
      m_drivetrainSubsystem = container.getDrivetrainSubsystem();
      m_intakeSubsystem = container.getIntakeSubsystem();
      m_shooterSubsystem = container.getShooterSubsystem();
      m_autonPath = autonpath;
      m_climbSubsystem = container.getClimbSubsystem();

      addCommands(
        sequence(
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
                new IntakeArmRollerCommand(m_intakeSubsystem, 0.55)
              ).withTimeout(1.5),
              new WaitCommand(0.2),
              new IntakePositionCommand(m_intakeSubsystem, Constants.INTAKE_ARM_POSITION_IN)
            )
          ),
          new AutonMacroPlayback( "/home/lvuser/turn180.csv", m_drivetrainSubsystem ),
          parallel(
            new AutoAimCommand( m_drivetrainSubsystem),
            new AutoDriveCommand( m_drivetrainSubsystem, 0.0, 0.0, 0.0 ),
            new KickerCommand( m_shooterSubsystem, -0.5, true, false, 0 ).withTimeout(0.5)
          ).withTimeout(1.5),
        parallel(
            new ShooterYeetCommandPart3ElectricBoogaloo(m_shooterSubsystem, Constants.YEET_SPEED_HIGH),
            new KickerMultipleCommand( m_shooterSubsystem, 0.7, m_intakeSubsystem )
          ).withTimeout(5.0)
        ).withTimeout(15.0)
      );
    }
  }
