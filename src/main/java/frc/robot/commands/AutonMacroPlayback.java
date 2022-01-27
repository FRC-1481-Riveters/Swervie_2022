package frc.robot.commands;

import frc.robot.subsystems.DrivetrainSubsystem;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.MacroPlayback;
import java.io.IOException;

public class AutonMacroPlayback extends CommandBase {

    private DrivetrainSubsystem m_drivetrain;
    private String m_filename;
    private MacroPlayback m_playback;

    public AutonMacroPlayback( String filename, DrivetrainSubsystem drivetrain )
    {
        super();
        m_filename = filename;
        m_drivetrain = drivetrain;
    }
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    super.initialize();
    try 
    {
        m_playback = new MacroPlayback( m_filename, m_drivetrain );
    }
    catch( IOException e )
    {
        e.printStackTrace();
    }
}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_playback.play();
    super.execute();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
      return m_playback.isFinished();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    super.end( interrupted );
  }

}
