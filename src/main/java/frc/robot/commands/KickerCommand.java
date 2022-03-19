package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ShooterSubsystem;

public class KickerCommand extends CommandBase {

    private ShooterSubsystem m_shooterSubsystem;
    private double m_output;
    private boolean m_useSensor;
    private boolean m_sensorActive;

    public KickerCommand( ShooterSubsystem subsystem, double value, boolean useSensor, boolean sensorActive)
    {
        m_shooterSubsystem = subsystem;
        m_output = value;
        m_useSensor = useSensor;
        m_sensorActive = sensorActive;
    }
    
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_shooterSubsystem.setKickerSpeed(m_output);
    super.initialize();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    super.execute();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if( m_useSensor == false )
    {
      return false;
    }
    else if(m_sensorActive == m_shooterSubsystem.getLightCurtain() ){
      return true;
    }
    else
    {
      return false;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_shooterSubsystem.setKickerSpeed(0);
    super.end(interrupted);
  }
}
