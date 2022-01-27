// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
//import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
//import edu.wpi.first.wpilibj2.command.FunctionalCommand;
//import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.Button;
import frc.robot.commands.DriveCommand;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ClimbSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.commands.AutonMacroPlayback;
import frc.robot.commands.AutonMacroRecord;

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
    m_drivetrainSubsystem.setDefaultCommand(new DriveCommand(
            m_drivetrainSubsystem,
            () -> -modifyAxis(m_controller.getLeftY() / joystickDivider) * DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND,
            () -> -modifyAxis(m_controller.getLeftX() / joystickDivider) * DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND,
            () -> -modifyAxis(m_controller.getRightX() / joystickDivider) * DrivetrainSubsystem.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND
    ));

    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    // Back button zeros the gyroscope
    new Button(m_controller::getBackButton)
            // No requirements because we don't need to interrupt anything
            .whenPressed(m_drivetrainSubsystem::zeroGyroscope);

    new Button(m_controller::getStartButton)
            .whenPressed( new AutonMacroRecord( "/home/lvuser/autonpath.csv", m_drivetrainSubsystem) );

  }

  public void checkBumper()
  {
    if(m_controller.getRightBumper()){
      joystickDivider = 1.0;
    }else{
      joystickDivider = 1.5;
    }
  }

  public void controlIntake(){
    m_intakeSubsystem.setIntakeSpeed(m_operatorController.getRightY() / 1.4);
  }

  public void controlClimb(){
    double climb6Speed;
    double climb10Speed;
    double climb15Speed;

    if(m_operatorController.getLeftBumper()==true){
      climb6Speed=0.3;
    }
    else if(m_operatorController.getLeftTriggerAxis()>=0.5){
      climb6Speed=-0.3;
    }
    else{
      climb6Speed=0;
    }


    if(m_operatorController.getRightBumper()==true){
      climb10Speed=0.3;
    }
    else if(m_operatorController.getRightTriggerAxis()>=0.5){
      climb10Speed=-0.3;
    }
    else{
      climb10Speed=0;
    }

    if(m_operatorController.getYButton()){
      climb15Speed=0.3;
    }
    else if(m_operatorController.getAButton()){
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
  if(m_controller.getYButton()){
    m_shooterSubsystem.setYeetSpeed(0.6);
  }/*Low A goal*/
  else if(m_controller.getXButton()){
    m_shooterSubsystem.setYeetSpeed(0.2);
  }/*B goal*/
  else if(m_controller.getBButton()){
    m_shooterSubsystem.setYeetSpeed(0.8);
  }/*C goal*/
  else if(m_controller.getAButton()){
    m_shooterSubsystem.setYeetSpeed(1.0);
  }
}

public void kickerPunt(){
  m_shooterSubsystem.setKickerSpeed(m_operatorController.getLeftY());
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

  private static double deadband(double value, double deadband) {
    if (Math.abs(value) > deadband) {
      if (value > 0.0) {
        return (value - deadband) / (1.0 - deadband);
      } else {
        return (value + deadband) / (1.0 - deadband);
      }
    } else {
      return 0.0;
    }
  }

  private static double modifyAxis(double value) {
    // Deadband
    value = deadband(value, 0.1);

    // Square the axis
    value = Math.copySign(value * value, value);

    return value;
  }
}
