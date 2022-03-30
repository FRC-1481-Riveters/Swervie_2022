package util;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.RobotContainer;
import frc.robot.commands.AutonMacroPlayback;
import frc.robot.commands.AutonPathDriveTurnShoot;
import frc.robot.commands.AutonPathShootBackup;
import common.control.Trajectory;
import common.math.RigidTransform2;
import common.math.Rotation2;

public class AutonomousChooser {
    private final AutonomousTrajectories trajectories;

    private SendableChooser<AutonomousMode> autonomousModeChooser = new SendableChooser<>();

    public AutonomousChooser(AutonomousTrajectories trajectories) {
        this.trajectories = trajectories;

        autonomousModeChooser.setDefaultOption("Auton nothing", AutonomousMode.AUTONOMOUS_NOTHING);
        autonomousModeChooser.addOption("Path A", AutonomousMode.AUTON_PATH_A);
        autonomousModeChooser.addOption("Path B", AutonomousMode.AUTON_PATH_B);
        autonomousModeChooser.addOption("Path C", AutonomousMode.AUTON_PATH_C);
        autonomousModeChooser.addOption("Shoot and Backup", AutonomousMode.AUTON_PATH_SHOOT_BACKUP);
        autonomousModeChooser.addOption("PlaybackSomething", AutonomousMode.PLAYBACK_SOMETHING);
    
        // Put the chooser on the dashboard
        SmartDashboard.putData( autonomousModeChooser );
    }

    public SendableChooser<AutonomousMode> getAutonomousModeChooser() {
        return autonomousModeChooser;
    }

    public Command getPlaybackSomethingCommand(RobotContainer robotContainer) {
        SequentialCommandGroup command = new SequentialCommandGroup();

        resetRobotPose(command, robotContainer, trajectories.getAutonPlaybackTrajectory());
        command.addCommands(new AutonMacroPlayback( "/home/lvuser/autonpath.csv", robotContainer.getDrivetrainSubsystem() ) );

        return command;
    }

    public Command getPathACommand(RobotContainer robotContainer) {
        SequentialCommandGroup command = new SequentialCommandGroup();

        resetRobotPose(command, robotContainer, trajectories.getAutonPlaybackTrajectory());
        command.addCommands(new AutonPathDriveTurnShoot( robotContainer, "/home/lvuser/driveforward40.csv" ) );

        return command;
    }

    public Command getPathBCommand(RobotContainer robotContainer) {
        SequentialCommandGroup command = new SequentialCommandGroup();

        resetRobotPose(command, robotContainer, trajectories.getAutonPlaybackTrajectory());
        command.addCommands(new AutonPathDriveTurnShoot( robotContainer, "/home/lvuser/driveforward50.csv" ) );

        return command;
    }

    public Command getPathCCommand(RobotContainer robotContainer) {
        SequentialCommandGroup command = new SequentialCommandGroup();

        resetRobotPose(command, robotContainer, trajectories.getAutonPlaybackTrajectory());
        command.addCommands(new AutonPathDriveTurnShoot( robotContainer, "/home/lvuser/pathc.csv" ) );

        return command;
    }

    public Command getPathBackupCommand(RobotContainer robotContainer){
        SequentialCommandGroup command = new SequentialCommandGroup();

        resetRobotPose(command, robotContainer, trajectories.getAutonPlaybackTrajectory());
        command.addCommands(new AutonPathShootBackup( robotContainer, "/home/lvuser/backup10foot.csv" ) );

        return command;
    }

    public Command AutonomousNothing(RobotContainer container) {
        SequentialCommandGroup command = new SequentialCommandGroup();

        command.addCommands(new WaitCommand(1.0) );

        return command;
    }

    public Command getCommand(RobotContainer robotContainer) {
        Command command;

        switch (autonomousModeChooser.getSelected()) {
            case AUTON_PATH_A:
                command = getPathACommand( robotContainer );
                break;

            case AUTON_PATH_B:
                command = getPathBCommand( robotContainer );
                break;

            case AUTON_PATH_C:
                command = getPathCCommand( robotContainer );
                break;

            case AUTON_PATH_SHOOT_BACKUP:
                command = getPathBackupCommand(robotContainer);
                break;

            case PLAYBACK_SOMETHING:
                command = getPlaybackSomethingCommand( robotContainer );
                break;

            default:
                command = new WaitCommand( 1.0 );
                break;
        }
        return command;
    }

    private void resetRobotPose(SequentialCommandGroup command, RobotContainer container, Trajectory trajectory) {
        command.addCommands(new InstantCommand(() -> container.getDrivetrainSubsystem().resetGyroAngle(Rotation2.ZERO)));
        command.addCommands(new InstantCommand(() -> container.getDrivetrainSubsystem().resetPose(
                new RigidTransform2(trajectory.calculate(0.0).getPathState().getPosition(), Rotation2.ZERO))));
     }
     
    private enum AutonomousMode {
        AUTONOMOUS_NOTHING,
        AUTON_PATH_A,
        AUTON_PATH_B,
        AUTON_PATH_C,
        AUTON_PATH_SHOOT_BACKUP,
        PLAYBACK_SOMETHING,
    }
}