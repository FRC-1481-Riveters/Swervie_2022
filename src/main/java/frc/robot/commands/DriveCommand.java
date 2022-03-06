package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.DrivetrainSubsystem;
import common.math.Vector2;
import common.robot.input.Axis;

public class DriveCommand extends CommandBase {
    private DrivetrainSubsystem drivetrainSubsystem;
    private Axis forward;
    private Axis strafe;
    private Axis rotation;
    private RobotContainer m_robotContainer;

    public DriveCommand(DrivetrainSubsystem drivetrain, Axis forward, Axis strafe, Axis rotation, RobotContainer rc) {
        this.forward = forward;
        this.strafe = strafe;
        this.rotation = rotation;
        this.m_robotContainer = rc;

        drivetrainSubsystem = drivetrain;

        addRequirements(drivetrain);
    }

    @Override
    public void execute() {
        double driveRotation;

        if( m_robotContainer.autoAimAngle != 0 )
        {
            driveRotation = Math.toRadians( m_robotContainer.autoAimAngle );
        }
        else
        {
            // !*!*!* FIXME: not sure why we have to divide rotate by 10 here...  maybe it should be radians instead of -1..1
            driveRotation = rotation.get(true) / 10.0;
        }
        drivetrainSubsystem.drive(new Vector2(forward.get(true)/drivetrainSubsystem.joystickDivider,
                                              strafe.get(true)/drivetrainSubsystem.joystickDivider),
                                      driveRotation, 
                                      true);
    }

    @Override
    public void end(boolean interrupted) {
        drivetrainSubsystem.drive(Vector2.ZERO, 0, false);
    }
}