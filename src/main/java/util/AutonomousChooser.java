package util;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.RobotContainer;
import frc.robot.commands.*;
import common.control.Trajectory;
import common.math.RigidTransform2;
import common.math.Rotation2;

public class AutonomousChooser {
    private final AutonomousTrajectories trajectories;

    private SendableChooser<AutonomousMode> autonomousModeChooser = new SendableChooser<>();

    public AutonomousChooser(AutonomousTrajectories trajectories) {
        this.trajectories = trajectories;

        autonomousModeChooser.setDefaultOption("Auton nothing", AutonomousMode.AUTONOMOUS_NOTHING);
        autonomousModeChooser.addOption("PlaybackSomething", AutonomousMode.PLAYBACK_SOMETHING);
    
        // Put the chooser on the dashboard
        SmartDashboard.putData( autonomousModeChooser );
    }

    public SendableChooser<AutonomousMode> getAutonomousModeChooser() {
        return autonomousModeChooser;
    }

    public Command getPlaybackSomethingCommand(RobotContainer container) {
        SequentialCommandGroup command = new SequentialCommandGroup();

        resetRobotPose(command, container, trajectories.getAutonPlaybackTrajectory());
        command.addCommands(new AutonMacroPlayback( "/home/lvuser/autonpath.csv", container.getDrivetrainSubsystem() ) );

        return command;
    }

    public Command AutonomousNothing(RobotContainer container) {
        SequentialCommandGroup command = new SequentialCommandGroup();

        command.addCommands(new WaitCommand(1.0) );

        return command;
    }

    public Command getCommand(RobotContainer container) {
        switch (autonomousModeChooser.getSelected()) {
            case PLAYBACK_SOMETHING:
                return getPlaybackSomethingCommand(container);
        }

        return getPlaybackSomethingCommand(container);
    }

    private void resetRobotPose(SequentialCommandGroup command, RobotContainer container, Trajectory trajectory) {
        command.addCommands(new InstantCommand(() -> container.getDrivetrainSubsystem().resetGyroAngle(Rotation2.ZERO)));
        command.addCommands(new InstantCommand(() -> container.getDrivetrainSubsystem().resetPose(
                new RigidTransform2(trajectory.calculate(0.0).getPathState().getPosition(), Rotation2.ZERO))));
     }
     
    private enum AutonomousMode {
        AUTONOMOUS_NOTHING,
        PLAYBACK_SOMETHING
    }
}