// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.sensors.Pigeon2;
import com.swervedrivespecialties.swervelib.Mk3SwerveModuleHelper;
import com.swervedrivespecialties.swervelib.SwerveModule;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj.Timer;

import common.control.*;
import common.drivers.Gyroscope;
import common.robot.drivers.Pigeon;
import common.robot.drivers.Pigeon.Axis;
import common.kinematics.ChassisVelocity;
import common.kinematics.SwerveKinematics;
import common.kinematics.SwerveOdometry;
import common.math.RigidTransform2;
import common.math.Rotation2;
import common.math.Vector2;
import common.robot.UpdateManager;
import common.util.*;

import java.util.Optional;

import static frc.robot.Constants.*;

public class DrivetrainSubsystem implements Subsystem, UpdateManager.Updatable {
  /**
   * The maximum voltage that will be delivered to the drive motors.
   * <p>
   * This can be reduced to cap the robot's maximum speed. Typically, this is useful during initial testing of the robot.
   */
  public static final double MAX_VOLTAGE = 12.0;
  // FIXME Measure the drivetrain's maximum velocity or calculate the theoretical.
  //  The formula for calculating the theoretical maximum velocity is:
  //   <Motor free speed RPM> / 60 * <Drive reduction> * <Wheel diameter meters> * pi
  //  By default this value is setup for a Mk3 standard module using Falcon500s to drive.
  //  An example of this constant for a Mk4 L2 module with NEOs to drive is:
  //   5880.0 / 60.0 / SdsModuleConfigurations.MK4_L2.getDriveReduction() * SdsModuleConfigurations.MK4_L2.getWheelDiameter() * Math.PI
  /**
   * The maximum velocity of the robot in meters per second.
   * <p>
   * This is a measure of how fast the robot should be able to drive in a straight line.
   */
  public static final double MAX_VELOCITY_INCHES_PER_SECOND = 120;
  public static final double MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND = MAX_VELOCITY_INCHES_PER_SECOND /
          Math.hypot(DRIVETRAIN_TRACKWIDTH_INCHES / 2.0, DRIVETRAIN_WHEELBASE_INCHES / 2.0);

  public static final DrivetrainFeedforwardConstants FEEDFORWARD_CONSTANTS = new DrivetrainFeedforwardConstants(
        1.0/(10.0*12.0),
        0.0032181,
        0.30764
  );

  public static final TrajectoryConstraint[] TRAJECTORY_CONSTRAINTS = {
        new FeedforwardConstraint(11.0, FEEDFORWARD_CONSTANTS.getVelocityConstant(), FEEDFORWARD_CONSTANTS.getAccelerationConstant(), false),
        new MaxAccelerationConstraint(12.5 * 12.0),
        new CentripetalAccelerationConstraint(20 * 12.0)
  };

  private static final int MAX_LATENCY_COMPENSATION_MAP_ENTRIES = 25;

  private final HolonomicMotionProfiledTrajectoryFollower follower = new HolonomicMotionProfiledTrajectoryFollower(
    new PidConstants(0.5, 0.0, 0.025),
    new PidConstants(5.0, 0.0, 0.0),
    new HolonomicFeedforward(FEEDFORWARD_CONSTANTS)
  );

  private final SwerveKinematics swerveKinematics = new SwerveKinematics(
        new Vector2(DRIVETRAIN_TRACKWIDTH_INCHES / 2.0, DRIVETRAIN_WHEELBASE_INCHES / 2.0),        //front left
        new Vector2(DRIVETRAIN_TRACKWIDTH_INCHES / 2.0, -DRIVETRAIN_WHEELBASE_INCHES / 2.0),       //front right
        new Vector2(-DRIVETRAIN_TRACKWIDTH_INCHES / 2.0, DRIVETRAIN_WHEELBASE_INCHES / 2.0),       //back left
        new Vector2(-DRIVETRAIN_TRACKWIDTH_INCHES / 2.0, -DRIVETRAIN_WHEELBASE_INCHES / 2.0)       //back right
);

  private final SwerveModule[] modules;

  private final Object sensorLock = new Object();
  //@GuardedBy("sensorLock")
  // By default we use a Pigeon for our gyroscope. But if you use another gyroscope, like a NavX, you can change this.
  // The important thing about how you configure your gyroscope is that rotating the robot counter-clockwise should
  // cause the angle reading to increase until it wraps back over to zero.
  private final Pigeon gyroscope = new Pigeon( 60 );

  private final Object kinematicsLock = new Object();
  //@GuardedBy("kinematicsLock")
  private final SwerveOdometry swerveOdometry = new SwerveOdometry(swerveKinematics, RigidTransform2.ZERO);
  //@GuardedBy("kinematicsLock")
  private RigidTransform2 pose = RigidTransform2.ZERO;
  //@GuardedBy("kinematicsLock")
  private final InterpolatingTreeMap<InterpolatingDouble, RigidTransform2> latencyCompensationMap = new InterpolatingTreeMap<>();
  //@GuardedBy("kinematicsLock")
  private Vector2 velocity = Vector2.ZERO;
  //@GuardedBy("kinematicsLock")
  private double angularVelocity = 0.0;

  private final Object stateLock = new Object();
  //@GuardedBy("stateLock")
  private HolonomicDriveSignal driveSignal = null;

  public  double autoAimAngle = 0;
  private double rollAngle;

  // Logging
  private final NetworkTableEntry odometryXEntry;
  private final NetworkTableEntry odometryYEntry;
  private final NetworkTableEntry odometryAngleEntry;
  private final NetworkTableEntry rollAngleEntry;

  // These are our modules. We initialize them in the constructor.
  public final SwerveModule frontLeftModule;
  public final SwerveModule frontRightModule;
  public final SwerveModule backLeftModule;
  public final SwerveModule backRightModule;

  // For macro record/playback
  public double m_forward;
  public double m_strafe;
  public double m_rotation;
  public double joystickDivider = 1.5;

  public DrivetrainSubsystem() {
        synchronized (sensorLock) {
                gyroscope.setInverted(true);
            }
        ShuffleboardTab shuffleboardTab = Shuffleboard.getTab("Drivetrain");

        frontLeftModule = Mk3SwerveModuleHelper.createFalcon500(
                shuffleboardTab.getLayout("Front Left Module", BuiltInLayouts.kList)
                        .withSize(2, 4)
                        .withPosition(0, 0),
                Mk3SwerveModuleHelper.GearRatio.STANDARD,
                FRONT_LEFT_MODULE_DRIVE_MOTOR,
                FRONT_LEFT_MODULE_STEER_MOTOR,
                FRONT_LEFT_MODULE_STEER_ENCODER,
                FRONT_LEFT_MODULE_STEER_OFFSET
        );

        frontRightModule = Mk3SwerveModuleHelper.createFalcon500(
                shuffleboardTab.getLayout("Front Right Module", BuiltInLayouts.kList)
                        .withSize(2, 4)
                        .withPosition(2, 0),
                Mk3SwerveModuleHelper.GearRatio.STANDARD,
                FRONT_RIGHT_MODULE_DRIVE_MOTOR,
                FRONT_RIGHT_MODULE_STEER_MOTOR,
                FRONT_RIGHT_MODULE_STEER_ENCODER,
                FRONT_RIGHT_MODULE_STEER_OFFSET
        );

        backLeftModule = Mk3SwerveModuleHelper.createFalcon500(
                shuffleboardTab.getLayout("Back Left Module", BuiltInLayouts.kList)
                        .withSize(2, 4)
                        .withPosition(4, 0),
                Mk3SwerveModuleHelper.GearRatio.STANDARD,
                BACK_LEFT_MODULE_DRIVE_MOTOR,
                BACK_LEFT_MODULE_STEER_MOTOR,
                BACK_LEFT_MODULE_STEER_ENCODER,
                BACK_LEFT_MODULE_STEER_OFFSET
        );

        backRightModule = Mk3SwerveModuleHelper.createFalcon500(
                shuffleboardTab.getLayout("Back Right Module", BuiltInLayouts.kList)
                        .withSize(2, 4)
                        .withPosition(6, 0),
                Mk3SwerveModuleHelper.GearRatio.STANDARD,
                BACK_RIGHT_MODULE_DRIVE_MOTOR,
                BACK_RIGHT_MODULE_STEER_MOTOR,
                BACK_RIGHT_MODULE_STEER_ENCODER,
                BACK_RIGHT_MODULE_STEER_OFFSET
        );

        modules = new SwerveModule[]{frontLeftModule, frontRightModule, backLeftModule, backRightModule};

        odometryXEntry = shuffleboardTab.add("X", 0.0)
                .withPosition(0, 0)
                .withSize(1, 1)
                .getEntry();
        odometryYEntry = shuffleboardTab.add("Y", 0.0)
                .withPosition(0, 1)
                .withSize(1, 1)
                .getEntry();
        odometryAngleEntry = shuffleboardTab.add("Angle", 0.0)
                .withPosition(0, 2)
                .withSize(1, 1)
                .getEntry();
        rollAngleEntry = shuffleboardTab.add("Roll Angle", 0.0).getEntry();
        shuffleboardTab.addNumber("Trajectory X", () -> {
                if (follower.getLastState() == null) {
                        return 0.0;
                }
                return follower.getLastState().getPathState().getPosition().x;
                })
                .withPosition(1, 0)
                .withSize(1, 1);
        shuffleboardTab.addNumber("Trajectory Y", () -> {
                if (follower.getLastState() == null) {
                        return 0.0;
                }
                return follower.getLastState().getPathState().getPosition().y;
                })
                .withPosition(1, 1)
                .withSize(1, 1);

        shuffleboardTab.addNumber("Rotation Voltage", () -> {
            HolonomicDriveSignal signal;
            synchronized (stateLock) {
                    signal = driveSignal;
        }

        if (signal == null) {
                return 0.0;
        }

        return signal.getRotation() * RobotController.getBatteryVoltage();
        });

        shuffleboardTab.addNumber("Average Velocity", this::getAverageAbsoluteValueVelocity);

}

public RigidTransform2 getPose() {
        synchronized (kinematicsLock) {
            return pose;
        }
    }

    public Vector2 getVelocity() {
        synchronized (kinematicsLock) {
            return velocity;
        }
    }

    public double getAngularVelocity() {
        synchronized (kinematicsLock) {
            return angularVelocity;
        }
    }

    public void drive(Vector2 translationalVelocity, double rotationalVelocity, boolean isFieldOriented) {
        synchronized (stateLock) {
            driveSignal = new HolonomicDriveSignal(translationalVelocity, rotationalVelocity, isFieldOriented);
        }
        m_forward = translationalVelocity.x;
        m_strafe = translationalVelocity.y;
        m_rotation = rotationalVelocity;
    }

    public void resetPose(RigidTransform2 pose) {
        synchronized (kinematicsLock) {
            this.pose = pose;
            swerveOdometry.resetPose(pose);
        }
    }

    public void resetGyroAngle(Rotation2 angle) {
        synchronized (sensorLock) {
            gyroscope.setAdjustmentAngle(
                    gyroscope.getUnadjustedAngle().rotateBy(angle.inverse())
            );
        }
    }

    public double getAverageAbsoluteValueVelocity() {
        double averageVelocity = 0;
        for (var module : modules) {
            averageVelocity += Math.abs(module.getDriveVelocity());
        }
        return averageVelocity / 4;
    }

    private void updateOdometry(double time, double dt) {
        Vector2[] moduleVelocities = new Vector2[modules.length];
        for (int i = 0; i < modules.length; i++) {
            var module = modules[i];

            moduleVelocities[i] = Vector2.fromAngle(Rotation2.fromRadians(module.getSteerAngle())).scale(module.getDriveVelocity() * 39.37008);
        }

        Rotation2 angle;
        double angularVelocity;
        synchronized (sensorLock) {
            angle = gyroscope.getAngle();
            angularVelocity = gyroscope.getRate();
        }

        ChassisVelocity velocity = swerveKinematics.toChassisVelocity(moduleVelocities);

        synchronized (kinematicsLock) {

            this.pose = swerveOdometry.update(angle, dt, moduleVelocities);
            if (latencyCompensationMap.size() > MAX_LATENCY_COMPENSATION_MAP_ENTRIES) {
                latencyCompensationMap.remove(latencyCompensationMap.firstKey());
            }
            latencyCompensationMap.put(new InterpolatingDouble(time), pose);
            this.velocity = velocity.getTranslationalVelocity();
            this.angularVelocity = angularVelocity;
        }
    }

    private void updateModules(HolonomicDriveSignal driveSignal, double dt) {
        ChassisVelocity chassisVelocity;
        if (driveSignal == null) {
            chassisVelocity = new ChassisVelocity(Vector2.ZERO, 0.0);
        } else if (driveSignal.isFieldOriented()) {
            chassisVelocity = new ChassisVelocity(
                    // FIXME:  ask 2910 whether this inverse was intentional
                    // FIXME:  because it screws up swerve orientation when turned by more than 45 degrees!
                    driveSignal.getTranslation().rotateBy(getPose().rotation), //.inverse()),
                    driveSignal.getRotation()
            );
        } else {
            chassisVelocity = new ChassisVelocity(
                    driveSignal.getTranslation(),
                    driveSignal.getRotation()
            );
        }

        Vector2[] moduleOutputs = swerveKinematics.toModuleVelocities(chassisVelocity);
        SwerveKinematics.normalizeModuleVelocities(moduleOutputs, 1);

        modules[0].set(-moduleOutputs[0].length * 12.0, moduleOutputs[0].getAngle().toRadians());
        modules[1].set(moduleOutputs[1].length * 12.0, moduleOutputs[1].getAngle().toRadians());
        modules[2].set(-moduleOutputs[2].length * 12.0, moduleOutputs[2].getAngle().toRadians());
        modules[3].set(moduleOutputs[3].length * 12.0, moduleOutputs[3].getAngle().toRadians());

        /*
        for (int i = 0; i < moduleOutputs.length; i++) {
            var module = modules[i];
            module.set(moduleOutputs[i].length * 12.0, moduleOutputs[i].getAngle().toRadians());
        }
    */
    }

    public RigidTransform2 getPoseAtTime(double timestamp) {
        synchronized (kinematicsLock) {
            if (latencyCompensationMap.isEmpty()) {
                return RigidTransform2.ZERO;
            }
            return latencyCompensationMap.getInterpolated(new InterpolatingDouble(timestamp));
        }
    }

    @Override
    public void update(double time, double dt) {
        updateOdometry(time, dt);

        HolonomicDriveSignal driveSignal;
        Optional<HolonomicDriveSignal> trajectorySignal = follower.update(
                getPose(),
                getVelocity(),
                getAngularVelocity(),
                time,
                dt
        );
        if (trajectorySignal.isPresent()) {
            driveSignal = trajectorySignal.get();
            driveSignal = new HolonomicDriveSignal(
                    driveSignal.getTranslation().scale(1.0 / RobotController.getBatteryVoltage()),
                    driveSignal.getRotation() / RobotController.getBatteryVoltage(),
                    driveSignal.isFieldOriented()
            );
        } else {
            synchronized (stateLock) {
                driveSignal = this.driveSignal;
            }
        }

        updateModules(driveSignal, dt);
    }

    private double lastTimestamp = 0.0;

    @Override
    public void periodic() {
        RigidTransform2 pose = getPose();
        odometryXEntry.setDouble(pose.translation.x);
        odometryYEntry.setDouble(pose.translation.y);
        odometryAngleEntry.setDouble(getPose().rotation.toDegrees());
        rollAngle = Math.toDegrees( gyroscope.getAxis(Axis.ROLL) );
        if( rollAngle < 0 )
            rollAngle = rollAngle + 90;
        else
            rollAngle = rollAngle - 90;
          rollAngleEntry.setDouble( rollAngle );
        final double timestamp = Timer.getFPGATimestamp();
        final double dt = timestamp - lastTimestamp;
        lastTimestamp = timestamp;
        update(timestamp, dt);
    }

    public HolonomicMotionProfiledTrajectoryFollower getFollower() {
        return follower;
    }

    public double gyroGetRoll()
    {
        return rollAngle;
    }
}