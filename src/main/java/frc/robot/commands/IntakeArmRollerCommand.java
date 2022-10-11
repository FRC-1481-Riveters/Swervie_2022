package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IntakeSubsystem;

public class IntakeArmRollerCommand extends CommandBase {

    private IntakeSubsystem m_intakeSubsystem;
    private double m_speed;

    public IntakeArmRollerCommand( IntakeSubsystem subsystem, double speed )
    {
        m_intakeSubsystem = subsystem;
        m_speed = speed;
    }

        // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        m_intakeSubsystem.setIntakeSpeed( m_speed );
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }

    // Returns true when the command should end.
    @Override
    public void end(boolean interrupted) {
        m_intakeSubsystem.setIntakeSpeed( 0.0 );
    }

}
