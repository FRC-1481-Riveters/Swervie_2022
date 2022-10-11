package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ShooterSubsystem;

public class ShooterCommand extends CommandBase {

    private ShooterSubsystem m_shooterSubsystem;
    private double m_output;

    public ShooterCommand( ShooterSubsystem subsystem, double value )
    {
        m_shooterSubsystem = subsystem;
        m_output = value;

        addRequirements(m_shooterSubsystem);
    }
    
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_shooterSubsystem.setYeetSpeed(m_output);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
      return false;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_shooterSubsystem.setYeetSpeed(0);
  }
}
