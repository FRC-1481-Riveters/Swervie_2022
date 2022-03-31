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
import frc.robot.commands.AutonPathDriveShoot;
import common.control.Trajectory;
import common.math.RigidTransform2;
import common.math.Rotation2;

public class AutonomousChooser {
    private final AutonomousTrajectories trajectories;

    private SendableChooser<AutonomousMode> autonomousModeChooser = new SendableChooser<>();

    public AutonomousChooser(AutonomousTrajectories trajectories) {
        this.trajectories = trajectories;

        autonomousModeChooser.setDefaultOption("Auton nothing", AutonomousMode.AUTONOMOUS_NOTHING);
        autonomousModeChooser.addOption("Auton Wall", AutonomousMode.AUTON_PATH_WALL);
        autonomousModeChooser.addOption("Auton Middle", AutonomousMode.AUTON_PATH_MIDDLE);
        autonomousModeChooser.addOption("Auton 3Ball", AutonomousMode.AUTON_PATH_3BALL);
        autonomousModeChooser.addOption("Auton Playback", AutonomousMode.PLAYBACK_SOMETHING);
    
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

    public Command getWallCommand(RobotContainer robotContainer) {
        SequentialCommandGroup command = new SequentialCommandGroup();

        resetRobotPose(command, robotContainer, trajectories.getAutonPlaybackTrajectory());
        command.addCommands(new AutonPathDriveTurnShoot( robotContainer, "/home/lvuser/deploy/driveforward40.csv" ) );

        return command;
    }

    public Command getMiddleCommand(RobotContainer robotContainer) {
        SequentialCommandGroup command = new SequentialCommandGroup();

        resetRobotPose(command, robotContainer, trajectories.getAutonPlaybackTrajectory());
        command.addCommands(new AutonPathDriveTurnShoot( robotContainer, "/home/lvuser/deploy/driveforward50.csv" ) );

        return command;
    }

    public Command get3BallCommand(RobotContainer robotContainer) {
        SequentialCommandGroup command = new SequentialCommandGroup();

        resetRobotPose(command, robotContainer, trajectories.getAutonPlaybackTrajectory());
        command.addCommands(new AutonPathDriveTurnShoot( robotContainer, "/home/lvuser/deploy/driveforward40.csv" ) );
        command.addCommands(new AutonPathDriveShoot( robotContainer, "/home/lvuser/deploy/ball3.csv" ) );

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
            case AUTON_PATH_WALL:
                command = getWallCommand( robotContainer );
                break;

            case AUTON_PATH_MIDDLE:
                command = getMiddleCommand( robotContainer );
                break;

            case AUTON_PATH_3BALL:
                command = get3BallCommand( robotContainer );
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
        AUTON_PATH_WALL,
        AUTON_PATH_MIDDLE,
        AUTON_PATH_3BALL,
        PLAYBACK_SOMETHING,
    }
}