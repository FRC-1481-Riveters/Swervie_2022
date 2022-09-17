package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ShooterSubsystem;

// This command waits until the shooter is up to speed.

public class ShooterWait extends CommandBase {

    private ShooterSubsystem m_shooterSubsystem;

    public ShooterWait( ShooterSubsystem subsystem )
    {
        m_shooterSubsystem = subsystem;
        addRequirements( m_shooterSubsystem );    
    }

    @Override
  public void initialize() {
   // System.out.println(System.currentTimeMillis() + " shooter wait");
  }
    

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
      if (m_shooterSubsystem.isAtSpeed()){
          //System.out.println(System.currentTimeMillis() + " at speed");

      }
      return m_shooterSubsystem.isAtSpeed();
      
  }
}
