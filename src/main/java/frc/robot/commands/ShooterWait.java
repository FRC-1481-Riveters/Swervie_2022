package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ShooterSubsystem;

// This command waits until the shooter is up to speed.

public class ShooterWait extends CommandBase {

    private ShooterSubsystem m_shooterSubsystem;

    public ShooterWait( ShooterSubsystem subsystem )
    {
        m_shooterSubsystem = subsystem;
    }
    

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
      return m_shooterSubsystem.isAtSpeed();
  }
}
