// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import frc.robot.commands.DriveCommand;
import util.AutonomousChooser;
import util.AutonomousTrajectories;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ClimbSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.commands.AutoAimCommand;
import frc.robot.commands.Autoclimb6Command;
import frc.robot.commands.Autoclimb6StartCommand;
import frc.robot.commands.Autoclimb10Command;
import frc.robot.commands.Autoclimb15Command;
import frc.robot.commands.AutonMacroRecord;
import frc.robot.commands.Climb6ManualCommand;
import frc.robot.commands.Climb10ManualCommand;
import frc.robot.commands.Climb15ManualCommand;
import frc.robot.commands.ClimbZeroPosition;
import frc.robot.commands.IntakePositionCommand;
import frc.robot.commands.IntakeRetractCommand;
import frc.robot.commands.KickerMultipleCommand;
import frc.robot.commands.KickerCommand;
import frc.robot.commands.ShooterCommand;
import frc.robot.commands.ShooterYeetCommandPart2ElectricBoogaloo;
import frc.robot.commands.ShooterYeetCommandPart3ElectricBoogaloo;
import common.math.Rotation2;
import common.robot.input.Axis;
import common.robot.input.XboxController;
import common.robot.input.DPadButton.Direction;

import java.io.IOException;

import javax.lang.model.util.ElementScanner6;

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

  private double intakeArmPosition=0;

  private JoystickTriggerPressed driverLeftTrigger;
  private JoystickTriggerPressed operatorLeftTrigger;
  private JoystickTriggerPressed operatorRightTrigger;
  private JoystickAxisUp operatorLeftAxisUp;
  private JoystickAxisDown operatorLeftAxisDown;
  public  double autoAimAngle = 0;

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

    CommandScheduler.getInstance().registerSubsystem(m_drivetrainSubsystem);
    CommandScheduler.getInstance().setDefaultCommand(m_drivetrainSubsystem, new DriveCommand(m_drivetrainSubsystem, getDriveForwardAxis(), getDriveStrafeAxis(), getDriveRotationAxis(), this));

    autonomousChooser = new AutonomousChooser(autonomousTrajectories);

    m_controller.getLeftXAxis().setInverted(true);
    m_controller.getRightXAxis().setInverted(true);

    // Configure the button bindings
    configureButtonBindings();
    m_intakeSubsystem.IntakeSubsystemInit();
    m_climbSubsystem.ClimbSubsystemInit();
  }

  public DrivetrainSubsystem getDrivetrainSubsystem()
  {
    return m_drivetrainSubsystem;
  }

  public IntakeSubsystem getIntakeSubsystem()
  {
    return m_intakeSubsystem;
  }

  public ShooterSubsystem getShooterSubsystem()
  {
    return m_shooterSubsystem;
  }

  public ClimbSubsystem getClimbSubsystem(){
    return m_climbSubsystem;
  }

private Axis getDriveForwardAxis() {
    return m_controller.getLeftYAxis();
}

private Axis getDriveStrafeAxis() {
    return m_controller.getLeftXAxis();
}

private Axis getDriveRotationAxis() {
    return m_controller.getRightXAxis();
}

private class JoystickTriggerPressed extends Trigger {
  private Axis m_axis;
  public JoystickTriggerPressed( Axis axis )
  {
    m_axis = axis;
  }
  @Override
  public boolean get() {
    if( m_axis.get() >= 0.25 )
    {
      return true;
    }
    else
    {
      return false;
    }
  }
}

private class JoystickAxisUp extends Trigger {
  private Axis m_axis;
  public JoystickAxisUp( Axis axis )
  {
    m_axis = axis;
  }
  @Override
  public boolean get() {
    if( m_axis.get() >= 0.5 )
    {
      return true;
    }
    else
    {
      return false;
    }
  }
}

private class JoystickAxisDown extends Trigger {
  private Axis m_axis;
  public JoystickAxisDown( Axis axis )
  {
    m_axis = axis;
  }
  @Override
  public boolean get() {
    if( m_axis.get() <= -0.5 )
    {
      return true;
    }
    else
    {
      return false;
    }
  }
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

    // Driver left trigger records a macro
    driverLeftTrigger = new JoystickTriggerPressed( m_controller.getLeftTriggerAxis() );
    driverLeftTrigger
      .whenActive( new AutonMacroRecord( "/home/lvuser/autonpath.csv", m_drivetrainSubsystem) );

    m_controller.getLeftBumperButton()
      .whileActiveOnce( new AutoAimCommand( this ) );

    // Climb6 manual retract
    operatorLeftTrigger = new JoystickTriggerPressed( m_operatorController.getLeftTriggerAxis() );
    m_operatorController.getBackButton() 
      .and( operatorLeftTrigger )
        .whileActiveOnce( new Climb6ManualCommand(m_climbSubsystem, -0.4) );

    // Climb6 manual extend
    m_operatorController.getBackButton() 
      .and( m_operatorController.getLeftBumperButton() )
      .whileActiveOnce( new Climb6ManualCommand( m_climbSubsystem, 0.4) );

    // Climb10 manual retract
    operatorRightTrigger = new JoystickTriggerPressed( m_operatorController.getRightTriggerAxis() );
    m_operatorController.getBackButton() 
      .and( operatorRightTrigger )
      .whileActiveOnce( new Climb10ManualCommand(m_climbSubsystem, -0.4) );

    // Climb10 manual extend
    m_operatorController.getBackButton() 
      .and( m_operatorController.getRightBumperButton() )
      .whileActiveOnce( new Climb10ManualCommand(m_climbSubsystem, 0.4) );

    // Climb15 manual retract
    m_operatorController.getBackButton() 
      .and( m_operatorController.getAButton() )
      .whileActiveOnce( new Climb15ManualCommand( m_climbSubsystem, -0.4) );

    // Climb15 manual extend
    m_operatorController.getBackButton() 
      .and( m_operatorController.getYButton() )
      .whileActiveOnce( new Climb15ManualCommand( m_climbSubsystem, 0.4) );

    //Autoclimb6 Start
    m_controller.getStartButton()
      .and(m_controller.getYButton())
      .whileActiveOnce(new Autoclimb6StartCommand(m_climbSubsystem, m_intakeSubsystem));

    //Autoclimb6
    m_operatorController.getStartButton()
      .and(m_operatorController.getXButton())
      .whileActiveOnce(new Autoclimb6Command(m_climbSubsystem));

    // Autoclimb10
    m_operatorController.getStartButton()
      .and( m_operatorController.getYButton() )
      .whileActiveOnce( new Autoclimb10Command(m_climbSubsystem) );

    // Autoclimb15
    m_operatorController.getStartButton()
      .and( m_operatorController.getBButton() )
      .whileActiveOnce( new Autoclimb15Command(m_climbSubsystem) );

    //Zero climb positions
    m_operatorController.getStartButton()
      .and(m_operatorController.getBackButton())
      .whileActiveOnce(new ClimbZeroPosition(m_climbSubsystem));

    //Shooter controls on operator controller
    // center to back bumper  12' 1" = 2500 rpm   limelight ty 0 degrees
    // center to back bumper: 13' 3" = 2500 rpm   limelight ty 4.00 degrees
    // center to back bumper: 15' 1" = 2900 rpm
    m_operatorController.getDPadButton(Direction.UP)
      .whileActiveOnce( new ShooterYeetCommandPart2ElectricBoogaloo( m_shooterSubsystem, 2200) );

    m_operatorController.getDPadButton(Direction.LEFT)
      .whileActiveOnce( new ShooterYeetCommandPart2ElectricBoogaloo( m_shooterSubsystem, 1300) );

    m_operatorController.getDPadButton(Direction.RIGHT)
      .whileActiveOnce( new ShooterYeetCommandPart2ElectricBoogaloo( m_shooterSubsystem, 2400) );

    m_operatorController.getDPadButton(Direction.DOWN)
      .whileActiveOnce( new ShooterYeetCommandPart2ElectricBoogaloo( m_shooterSubsystem, 2600) );

    //Kicker controls on drive controller
    m_controller.getDPadButton(Direction.UP)
      .whileActiveOnce( new KickerCommand( m_shooterSubsystem, 1.0, false, false, 0 ) );

    m_controller.getDPadButton(Direction.DOWN)
      .whileActiveOnce(new KickerCommand(m_shooterSubsystem, -1.0, false, false, 0));

    m_controller.getAButton()
      .whileActiveOnce( new KickerMultipleCommand( m_shooterSubsystem, 0.7, m_intakeSubsystem ) );

    operatorLeftTrigger = new JoystickTriggerPressed( m_operatorController.getLeftTriggerAxis() );
    m_operatorController.getBackButton() 
      .and( operatorLeftTrigger )
        .whileActiveOnce( new Climb6ManualCommand(m_climbSubsystem, -0.4) );

    operatorLeftAxisUp = new JoystickAxisUp( m_operatorController.getLeftYAxis() );
    operatorLeftAxisUp
      .whileActiveOnce( new IntakePositionCommand( m_intakeSubsystem, Constants.INTAKE_ARM_POSITION_OUT ) );

    operatorLeftAxisDown = new JoystickAxisDown( m_operatorController.getLeftYAxis() );
    operatorLeftAxisDown
      .whileActiveOnce(  new IntakeRetractCommand( m_intakeSubsystem ) );
  }

  public void checkBumper()
  {
    if(m_controller.getRightBumperButton().get()==true){
      m_drivetrainSubsystem.joystickDivider = 1.5;
    }else{
      m_drivetrainSubsystem.joystickDivider = 1.0;
    }
  }

  public void controlIntake(){
    m_intakeSubsystem.setIntakeSpeed(m_operatorController.getRightYAxis().get() / 1.5 );
  }


  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return autonomousChooser.getCommand(this);
  }

}
