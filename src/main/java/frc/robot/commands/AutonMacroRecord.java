package frc.robot.commands;

import frc.robot.subsystems.DrivetrainSubsystem;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.MacroRecorder;
import java.io.IOException;

public class AutonMacroRecord extends CommandBase {

    private DrivetrainSubsystem m_drivetrain;
    private String m_filename;
    private MacroRecorder m_recorder;

    public AutonMacroRecord( String filename, DrivetrainSubsystem drivetrain )
    {
        m_filename = filename;
        m_drivetrain = drivetrain;
    }
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    try 
    {
        m_recorder = new MacroRecorder( m_filename, m_drivetrain );
    }
    catch( IOException e )
    {
        e.printStackTrace();
    }
}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    try 
    {
        m_recorder.record();
    }
    catch( IOException e )
    {
        e.printStackTrace();
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    try 
    {
        m_recorder.end();
    }
    catch( IOException e )
    {
        e.printStackTrace();
    }
  }

}
