package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DrivetrainSubsystem;
import common.math.Vector2;
import common.robot.input.Axis;

public class AutoDriveCommand extends CommandBase {
    private DrivetrainSubsystem m_drivetrainSubsystem;
    private double forward;
    private double strafe;
    private double rotation;

    public AutoDriveCommand(DrivetrainSubsystem drivetrain, double forward, double strafe, double rotation ) {
        this.forward = forward;
        this.strafe = strafe;
        this.rotation = rotation;

        m_drivetrainSubsystem = drivetrain;

        addRequirements(drivetrain);
    }

    @Override
    public void execute() {
        double driveRotation;

        if( m_drivetrainSubsystem.autoAimAngle != 0 )
        {
            driveRotation = Math.toRadians( m_drivetrainSubsystem.autoAimAngle );
        }
        else
        {
            driveRotation = rotation;
        }
        m_drivetrainSubsystem.drive(new Vector2( forward, strafe ), driveRotation, true);
    }

    @Override
    public void end(boolean interrupted) {
        m_drivetrainSubsystem.drive(Vector2.ZERO, 0, false);
    }
}