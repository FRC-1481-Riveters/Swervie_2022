package util;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.commands.AutonMacroPlayback;
import frc.robot.commands.FollowTrajectoryCommand;
import frc.robot.commands.Climb6PositionCommand;
import frc.robot.commands.Climb10PositionCommand;
import frc.robot.commands.Climb15PositionCommand;
import frc.robot.commands.IntakeArmRollerCommand;
import frc.robot.commands.IntakePositionCommand;
import frc.robot.commands.KickerCommand;
import frc.robot.commands.AutoAimCommand;
import frc.robot.commands.AutoDriveCommand;
import frc.robot.commands.ShooterYeetCommandPart3ElectricBoogaloo;

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
        autonomousModeChooser.addOption("Pathfinder Test", AutonomousMode.PATHFINDER_TEST);
    
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
        command.addCommands( AutonPathDriveTurnShoot( robotContainer, "/home/lvuser/deploy/driveforward40.csv" ) );

        return command;
    }

    public Command getMiddleCommand(RobotContainer robotContainer) {
        SequentialCommandGroup command = new SequentialCommandGroup();

        resetRobotPose(command, robotContainer, trajectories.getAutonPlaybackTrajectory());
        command.addCommands( AutonPathDriveTurnShoot( robotContainer, "/home/lvuser/deploy/driveforward50.csv" ) );

        return command;
    }

    public Command get3BallCommand(RobotContainer robotContainer) {
        SequentialCommandGroup command = new SequentialCommandGroup();

        resetRobotPose(command, robotContainer, trajectories.getAutonPlaybackTrajectory());
        command.addCommands( AutonPathDriveTurnShoot( robotContainer, "/home/lvuser/deploy/driveforward40.csv" ) );
        command.addCommands( AutonPathDriveShoot( robotContainer, "/home/lvuser/deploy/ball3.csv" ) );

        return command;
    }

    private Command getPathfinderTest(RobotContainer container) {
        SequentialCommandGroup command = new SequentialCommandGroup();

        //reset robot pose
        resetRobotPose(command, container, trajectories.getPathfinderTestPartOne());
//      command.addCommands(new HomeHoodMotorCommand(container.getShooterSubsystem()));
        //follow first trajectory and shoot
        follow(command, container, trajectories.getPathfinderTestPartOne());
//        shootAtTarget(command, container, 1.5);
        //follow second trajectory and shoot
        follow(command, container, trajectories.getPathfinderTestPartTwo());

//        follow(command, container, trajectories.getEightBallCompatiblePartThree());
//        shootAtTarget(command, container, 1.5);
//        follow(command, container, trajectories.getEightBallCompatiblePartFour());

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

            case PATHFINDER_TEST:
                command = getPathfinderTest( robotContainer );
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
        PATHFINDER_TEST
    }

    private void follow(SequentialCommandGroup command, RobotContainer container, Trajectory trajectory) {
        command.addCommands(new FollowTrajectoryCommand(container.getDrivetrainSubsystem(), trajectory)
//               .deadlineWith(new TargetWithShooterCommand(container.getShooterSubsystem(), container.getVisionSubsystem(), container.getPrimaryController()))
//                .alongWith(new PrepareBallsToShootCommand(container.getFeederSubsystem(), 1.0))
                );
    }

    public Command AutonPathDriveTurnShoot( RobotContainer container, String autonpath )
    {
        return 
            new SequentialCommandGroup(
                new ParallelCommandGroup(
                    new Climb6PositionCommand( container.getClimbSubsystem(), 600),
                    new Climb10PositionCommand(container.getClimbSubsystem(), 600),
                    //new Climb15PositionCommand(m_climbSubsystem, 600),
                    new AutonMacroPlayback( autonpath, container.getDrivetrainSubsystem() ),
                    new SequentialCommandGroup(
                        new ParallelCommandGroup(
                            new IntakePositionCommand(container.getIntakeSubsystem(), Constants.INTAKE_ARM_POSITION_OUT),
                            new IntakeArmRollerCommand(container.getIntakeSubsystem(), 0.60)
                        ).withTimeout(1.7),
                        new WaitCommand(0.2),
                        new IntakePositionCommand(container.getIntakeSubsystem(), Constants.INTAKE_ARM_POSITION_IN_FULL)
                    )
                ),
                new ParallelCommandGroup(
                    new AutonMacroPlayback( "/home/lvuser/deploy/turn180.csv", container.getDrivetrainSubsystem() ),
                    new SequentialCommandGroup(
                    new WaitCommand(0.5),
                    new KickerCommand( container.getShooterSubsystem(), -0.5, true, false, 0 ).withTimeout(0.5)
                    )
                ),
                new ParallelCommandGroup(
                    new AutoAimCommand( container.getDrivetrainSubsystem()).withTimeout(1.5),
                    new AutoDriveCommand( container.getDrivetrainSubsystem(), 0.0, 0.0, 0.0 ).withTimeout(1.5),
                    new ShooterYeetCommandPart3ElectricBoogaloo(container.getShooterSubsystem(), Constants.YEET_SPEED_HIGH),
                    new SequentialCommandGroup(
                        new WaitCommand(1.5),
                        container.KickerMultipleCommand( 0.7 )
                    )
                ).withTimeout(5.0)
            ).withTimeout(15.0);
    }

    public Command AutonPathDriveShoot( RobotContainer container, String autonpath )
    {
        return 
            new SequentialCommandGroup(
                new ParallelCommandGroup(
                    new AutonMacroPlayback( autonpath, container.getDrivetrainSubsystem()),
                    new SequentialCommandGroup(
                        new ParallelCommandGroup(
                            new IntakePositionCommand(container.getIntakeSubsystem(), Constants.INTAKE_ARM_POSITION_OUT),
                            new IntakeArmRollerCommand(container.getIntakeSubsystem(), 0.65)
                        ).withTimeout(5.3),
                        new IntakePositionCommand(container.getIntakeSubsystem(), Constants.INTAKE_ARM_POSITION_IN_FULL)
                    )
                ),
                new ParallelCommandGroup(
                    new AutoAimCommand( container.getDrivetrainSubsystem()).withTimeout(1.5),
                    new AutoDriveCommand( container.getDrivetrainSubsystem(), 0.0, 0.0, 0.0 ).withTimeout(1.5),
                    new ShooterYeetCommandPart3ElectricBoogaloo(container.getShooterSubsystem(), Constants.YEET_SPEED_HIGH),
                    new SequentialCommandGroup(
                        new WaitCommand(1.5),
                        container.KickerMultipleCommand( 0.7 )
                    )
                ).withTimeout(5.0)
            );
    }

}