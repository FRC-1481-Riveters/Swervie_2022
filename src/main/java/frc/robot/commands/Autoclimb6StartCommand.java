package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.ClimbSubsystem;

public class Autoclimb6StartCommand extends SequentialCommandGroup {

    private ClimbSubsystem m_climbSubsystem;

    public Autoclimb6StartCommand( ClimbSubsystem subsystem )
    {
      m_climbSubsystem = subsystem;
      addCommands(
        // 51400 = full travel
        new Climb6PositionCommand(m_climbSubsystem, -3400)
        .withTimeout(2.0),
        new Climb6PositionCommand(m_climbSubsystem, 51400)
        .withTimeout(5.0)
        );
    }

    @Override
    public void initialize() {
//      System.out.println("Autoclimb6Start initialize");
      NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setNumber(0); // force light off
      super.initialize();
    }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_climbSubsystem.setClimb6Speed(0);
    super.end(interrupted);
  }
}
