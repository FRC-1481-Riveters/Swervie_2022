package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import edu.wpi.first.networktables.NetworkTableInstance;

public class AutoAimCommand extends CommandBase {

    private RobotContainer m_robotContainer;

    public AutoAimCommand( RobotContainer rc )
    {
        m_robotContainer = rc;
    }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double angle;
    angle = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx").getDouble(0); // get target angle
    angle = angle * -0.1;
    if( Math.abs(angle) < 0.2 ) angle = 0;
    m_robotContainer.autoAimAngle = angle * 0.2;
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_robotContainer.autoAimAngle = 0;
    super.end( interrupted );
  }

}
