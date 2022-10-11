package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ShooterSubsystem;

public class KickerCommand extends CommandBase {

    private ShooterSubsystem m_shooterSubsystem;
    private double m_output;
    private boolean m_useSensor;
    private boolean m_sensorBlocked;
    private long m_startTime;
    private long m_minTime;

    public KickerCommand( ShooterSubsystem subsystem, double value, boolean useSensor, boolean sensorBlocked, long minTime)
    {
        m_shooterSubsystem = subsystem;
        m_output = value;
        m_useSensor = useSensor;
        m_sensorBlocked = sensorBlocked;
        m_minTime = minTime;

        addRequirements(m_shooterSubsystem);
    }
    
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_startTime = System.currentTimeMillis();
    m_shooterSubsystem.setKickerSpeed(m_output);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if( m_useSensor == false )
    {
      return false;
    }
    else if( (m_minTime == 0) || ((System.currentTimeMillis() - m_startTime) > m_minTime) )
    {
      if( m_sensorBlocked == m_shooterSubsystem.isLightCurtainBlocked() )
      {
        return true;
      }
      else
      {
        return false;
      }
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
  }
}
