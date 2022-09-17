package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IntakeSubsystem;

public class IntakePositionCommand extends CommandBase {

    private IntakeSubsystem m_intakeSubsystem;
    private double m_position;

    public IntakePositionCommand( IntakeSubsystem subsystem, double position )
    {
        m_intakeSubsystem = subsystem;
        m_position = position;

        addRequirements(m_intakeSubsystem);
    }

        // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        m_intakeSubsystem.setIntakeArmPosition( m_position );
    }
}
