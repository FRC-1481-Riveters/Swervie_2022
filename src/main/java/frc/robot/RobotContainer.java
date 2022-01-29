// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

//import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
//import edu.wpi.first.wpilibj2.command.FunctionalCommand;
//import edu.wpi.first.wpilibj2.command.InstantCommand;
//import edu.wpi.first.wpilibj2.command.button.Button;
import frc.robot.commands.DriveCommand;
import util.AutonomousChooser;
import util.AutonomousTrajectories;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ClimbSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.commands.AutonMacroPlayback;
import frc.robot.commands.AutonMacroRecord;
import common.math.Rotation2;
import common.math.Vector2;
import common.robot.input.Axis;
import common.robot.input.DPadButton;
import common.robot.input.XboxController;
import java.io.IOException;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final DrivetrainSubsystem m_drivetrainSubsystem = new DrivetrainSubsystem();
  private final IntakeSubsystem m_intakeSubsystem = new IntakeSubsystem();
  private final ClimbSubsystem m_climbSubsystem = new ClimbSubsystem();
  private final ShooterSubsystem m_shooterSubsystem = new ShooterSubsystem();

  private final XboxController m_controller = new XboxController(0);
  private final XboxController m_operatorController = new XboxController(1);

  private AutonomousTrajectories autonomousTrajectories;
  private final AutonomousChooser autonomousChooser;

  private double joystickDivider = 1.5;

  private final Command AutonPlayback =
  new AutonMacroPlayback( "/home/lvuser/autonpath.csv", m_drivetrainSubsystem );

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Set up the default command for the drivetrain.
    // The controls are for field-oriented driving:
    // Left stick Y axis -> forward and backwards movement
    // Left stick X axis -> left and right movement
    // Right stick X axis -> rotation
    try {
      autonomousTrajectories = new AutonomousTrajectories(DrivetrainSubsystem.TRAJECTORY_CONSTRAINTS);
  } catch (IOException e) {
      e.printStackTrace();
  }
  autonomousChooser = new AutonomousChooser(autonomousTrajectories);

  m_controller.getLeftXAxis().setInverted(true);
  m_controller.getRightXAxis().setInverted(true);

    CommandScheduler.getInstance().registerSubsystem(m_drivetrainSubsystem);
    CommandScheduler.getInstance().setDefaultCommand(m_drivetrainSubsystem, new DriveCommand(m_drivetrainSubsystem, getDriveForwardAxis(), getDriveStrafeAxis(), getDriveRotationAxis()));

    // Configure the button bindings
    configureButtonBindings();
  }

  public DrivetrainSubsystem getDrivetrainSubsystem()
  {
    return m_drivetrainSubsystem;
  }

  private double getDriveForwardAxis() {
    return m_controller.getLeftYAxis().get(true) / joystickDivider;
}

private double getDriveStrafeAxis() {
    return m_controller.getLeftXAxis().get(true) / joystickDivider;
}

private double getDriveRotationAxis() {
    return m_controller.getRightXAxis().get(true);
}

/**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    // Back button zeros the gyroscope
    m_controller.getBackButton().whenPressed(
      () -> m_drivetrainSubsystem.resetGyroAngle(Rotation2.ZERO) );

    m_controller.getStartButton().whenPressed(
      new AutonMacroRecord( "/home/lvuser/autonpath.csv", m_drivetrainSubsystem) );

  }

  public void checkBumper()
  {
    if(m_controller.getRightBumperButton().get()){
      joystickDivider = 1.0;
    }else{
      joystickDivider = 1.5;
    }
  }

  public void controlIntake(){
    m_intakeSubsystem.setIntakeSpeed(m_operatorController.getRightYAxis().get() / 1.4);
  }

  public void controlClimb(){
    double climb6Speed;
    double climb10Speed;
    double climb15Speed;

    if(m_operatorController.getLeftBumperButton().getAsBoolean()==true){
      climb6Speed=0.3;
    }
    else if(m_operatorController.getLeftTriggerAxis().get()>=0.5){
      climb6Speed=-0.3;
    }
    else{
      climb6Speed=0;
    }


    if(m_operatorController.getRightBumperButton().getAsBoolean()==true){
      climb10Speed=0.3;
    }
    else if(m_operatorController.getRightTriggerAxis().get()>=0.5){
      climb10Speed=-0.3;
    }
    else{
      climb10Speed=0;
    }

    if(m_operatorController.getYButton().getAsBoolean()){
      climb15Speed=0.3;
    }
    else if(m_operatorController.getAButton().getAsBoolean()){
      climb15Speed=-0.3;
    }
    else{
      climb15Speed=0;
    }
    m_climbSubsystem.setClimb6Speed(climb6Speed);
    m_climbSubsystem.setClimb10Speed(climb10Speed);
    m_climbSubsystem.setClimb15Speed(climb15Speed);
  }

public void shooterYeet(){
  /*High A goal*/
  if(m_controller.getYButton().getAsBoolean()){
    m_shooterSubsystem.setYeetSpeed(0.6);
  }/*Low A goal*/
  else if(m_controller.getXButton().getAsBoolean()){
    m_shooterSubsystem.setYeetSpeed(0.2);
  }/*B goal*/
  else if(m_controller.getBButton().getAsBoolean()){
    m_shooterSubsystem.setYeetSpeed(0.8);
  }/*C goal*/
  else if(m_controller.getAButton().getAsBoolean()){
    m_shooterSubsystem.setYeetSpeed(1.0);
  }
}

public void kickerPunt(){
  m_shooterSubsystem.setKickerSpeed(m_operatorController.getLeftYAxis().get());
}

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return AutonPlayback;
  }

}
