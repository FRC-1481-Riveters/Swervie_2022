package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ShooterSubsystem;

//yeet means to throw (cargo)
//electric boogaloo: idk it's a meme ig

public class ShooterYeetCommandPart3ElectricBoogaloo extends CommandBase{
    private ShooterSubsystem m_shooterSubsystem;
    private double m_shooterYeetSpeed;
    public ShooterYeetCommandPart3ElectricBoogaloo(ShooterSubsystem subsystem, double shooterIntendedSpeed) {
        m_shooterSubsystem = subsystem;
//        addRequirements( m_shooterSubsystem );    
        m_shooterYeetSpeed = shooterIntendedSpeed;
      }
    
      // Called when the command is initially scheduled.
      @Override
      public void initialize() {
        m_shooterSubsystem.setYeetSpeed(m_shooterYeetSpeed);
      }
    
      // Called once the command ends or is interrupted.
      @Override
      public void end(boolean interrupted) {
        m_shooterSubsystem.setYeetSpeed(0.0);
      }
    
      // Returns true when the command should end.
      @Override
      public boolean isFinished() {
        return false;
      }
    }
