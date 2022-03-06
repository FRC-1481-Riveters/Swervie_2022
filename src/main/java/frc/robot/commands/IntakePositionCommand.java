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
    }

        // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        System.out.println("SetIntakeArmPosition " + m_position);
        m_intakeSubsystem.setIntakeArmPosition( m_position );
        super.initialize();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return true;
    }

}
