package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ClimbSubsystem;

public class ClimbZeroPosition extends CommandBase {

    private ClimbSubsystem m_climbSubsystem;

    public ClimbZeroPosition( ClimbSubsystem subsystem )
    {
        m_climbSubsystem = subsystem;
    }
    
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println("****************** ZERO CLIMB SENSORS ******************");
    m_climbSubsystem.zeroClimbSensors();
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
    return true;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    super.end(interrupted);
  }
}
