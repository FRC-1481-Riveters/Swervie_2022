package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.commands.KickerCommand;
import frc.robot.commands.ShooterWait;

public class ShooterYeetCommandPart2ElectricBoogaloo extends SequentialCommandGroup {

  private ShooterSubsystem m_shooterSubsystem;
  private double shooterIntendedSpeed;

  public ShooterYeetCommandPart2ElectricBoogaloo( ShooterSubsystem shooterSubsystem, double speed )
  {
      m_shooterSubsystem = shooterSubsystem;
      shooterIntendedSpeed = speed;

//      addRequirements( m_shooterSubsystem );    

      addCommands(
          //new KickerCommand( m_shooterSubsystem, -0.5, true, false, 0 ).withTimeout(0.5),
          new ShooterYeetCommandPart3ElectricBoogaloo(m_shooterSubsystem, shooterIntendedSpeed)
      );
  
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_shooterSubsystem.setKickerSpeed(0.0);
    m_shooterSubsystem.setYeetSpeed(0.0);
  }
    
}
