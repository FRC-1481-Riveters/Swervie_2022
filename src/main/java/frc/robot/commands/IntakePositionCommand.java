package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IntakeSubsystem;

public class IntakePositionCommand extends CommandBase {

    private IntakeSubsystem m_intakeSubsystem;
    private double m_position;
    private boolean m_oneShot;

    public IntakePositionCommand( IntakeSubsystem subsystem, double position )
    {
        m_intakeSubsystem = subsystem;
        m_position = position;
    }

        // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        m_oneShot = false;
        super.initialize();
    }

    @Override
    public void execute() {
        m_intakeSubsystem.setIntakeArmPosition( m_position );
        m_oneShot = true;
        super.execute();
    }
    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return m_oneShot;
    }

    @Override
    public void end( boolean interrupted )
    {
        super.end( interrupted );
    }
}
