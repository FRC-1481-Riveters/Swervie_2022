package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.ClimbSubsystem;

public class AutonPathDriveShoot extends SequentialCommandGroup {

    private DrivetrainSubsystem m_drivetrainSubsystem;
    private IntakeSubsystem m_intakeSubsystem;
    private ShooterSubsystem m_shooterSubsystem;
    private String m_autonPath;
    private ClimbSubsystem m_climbSubsystem;

    public AutonPathDriveShootOld( RobotContainer container, String autonpath )
    {
      m_drivetrainSubsystem = container.getDrivetrainSubsystem();
      m_intakeSubsystem = container.getIntakeSubsystem();
      m_shooterSubsystem = container.getShooterSubsystem();
      m_climbSubsystem = container.getClimbSubsystem();
      m_autonPath = autonpath;

      addRequirements(m_drivetrainSubsystem, m_intakeSubsystem, m_shooterSubsystem, m_climbSubsystem);

      addCommands(
        parallel(
          new AutonMacroPlayback( m_autonPath, m_drivetrainSubsystem ),
          sequence(
            parallel(
              new IntakePositionCommand(m_intakeSubsystem, Constants.INTAKE_ARM_POSITION_OUT),
              new IntakeArmRollerCommand(m_intakeSubsystem, 0.65)
            ).withTimeout(5.3),
            new IntakePositionCommand(m_intakeSubsystem, Constants.INTAKE_ARM_POSITION_IN_FULL)
          )
        ),
        parallel(
          new AutoAimCommand( m_drivetrainSubsystem).withTimeout(1.5),
          new AutoDriveCommand( m_drivetrainSubsystem, 0.0, 0.0, 0.0 ).withTimeout(1.5),
          new ShooterYeetCommandPart3ElectricBoogaloo(m_shooterSubsystem, Constants.YEET_SPEED_HIGH),
          sequence(
            new WaitCommand(1.5),
            new KickerMultipleCommandOld( m_shooterSubsystem, 0.7 )
          )
        ).withTimeout(5.0)
      );
    }
  }
