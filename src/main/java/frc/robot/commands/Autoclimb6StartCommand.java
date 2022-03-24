package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.robot.Constants;
import frc.robot.subsystems.ClimbSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

public class Autoclimb6StartCommand extends SequentialCommandGroup {

    private ClimbSubsystem m_climbSubsystem;
    private IntakeSubsystem m_intakeSubsystem;

    public Autoclimb6StartCommand( ClimbSubsystem subsystem, IntakeSubsystem intakeSubsystem )
    {
      m_climbSubsystem = subsystem;
      m_intakeSubsystem = intakeSubsystem;

      addRequirements(m_climbSubsystem);

      addCommands(
        new IntakePositionCommand( m_intakeSubsystem, Constants.INTAKE_ARM_POSITION_IN_FULL ),
        // 51400 = full travel
        new Climb6PositionCommand(m_climbSubsystem, -3400)
        .withTimeout(2.0),
        new Climb6PositionCommand(m_climbSubsystem, 51400)
        .withTimeout(5.0)
      );

    }

    @Override
    public void initialize() {
      NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setNumber(0); // force light off
      super.initialize();
    }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_climbSubsystem.setClimb6Speed(0);
    m_climbSubsystem.setClimb10Speed(0);
  }
}
