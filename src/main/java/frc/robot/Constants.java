// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    /**
     * The left-to-right distance between the drivetrain wheels
     *
     * Should be measured from center to center.
     */
    public static final double DRIVETRAIN_TRACKWIDTH_INCHES = 20.5;
    /**
     * The front-to-back distance between the drivetrain wheels.
     *
     * Should be measured from center to center.
     */
    public static final double DRIVETRAIN_WHEELBASE_INCHES = 20.5;

    public static final int FRONT_LEFT_MODULE_DRIVE_MOTOR = 15;
    public static final int FRONT_LEFT_MODULE_STEER_MOTOR = 13;
    public static final int FRONT_LEFT_MODULE_STEER_ENCODER = 14;
    public static final double FRONT_LEFT_MODULE_STEER_OFFSET = -Math.toRadians(175.9); // FIXME Measure and set front left steer offset

    public static final int FRONT_RIGHT_MODULE_DRIVE_MOTOR = 12;
    public static final int FRONT_RIGHT_MODULE_STEER_MOTOR = 10;
    public static final int FRONT_RIGHT_MODULE_STEER_ENCODER = 11;
    public static final double FRONT_RIGHT_MODULE_STEER_OFFSET = -Math.toRadians(143.7); // FIXME Measure and set front right steer offset

    public static final int BACK_LEFT_MODULE_DRIVE_MOTOR = 18;
    public static final int BACK_LEFT_MODULE_STEER_MOTOR = 16;
    public static final int BACK_LEFT_MODULE_STEER_ENCODER = 17;
    public static final double BACK_LEFT_MODULE_STEER_OFFSET = -Math.toRadians(201.7); // FIXME Measure and set back left steer offset

    public static final int BACK_RIGHT_MODULE_DRIVE_MOTOR = 21;
    public static final int BACK_RIGHT_MODULE_STEER_MOTOR = 19;
    public static final int BACK_RIGHT_MODULE_STEER_ENCODER = 20;
    public static final double BACK_RIGHT_MODULE_STEER_OFFSET = -Math.toRadians(165.0); // FIXME Measure and set back right steer offset

    public static final int INTAKE_ARM_MOTOR = 7;
    public static final int INTAKE_MOTOR = 8;

    public static final int CLIMB_6_MOTOR = 30;
    public static final int CLIMB_10_MOTOR = 31;
    public static final int CLIMB_15_MOTOR = 32;

    public static final int CLIMB_6_ENCODER = 33;
    public static final int CLIMB_10_ENCODER = 34;
    public static final int CLIMB_15_ENCODER = 35;

    
    public static final int KICKER_MOTOR = 41;
    public static final int KICKER_ENCODER = 42;

    public static final int YEET_MOTOR = 40;
    public static final int YEET_SPEED_HIGH = 2500;
    public static final int YEET_SPEED_LOW = 1300;
    public static final double YEET_SPEED_TOLERANCE = 0.015;

    public static final int TALON_TIMEOUT_MS = 5000;

    public static final double INTAKE_ARM_MOTOR_KF = 0.07;
    public static final double INTAKE_ARM_MOTOR_KP = 1.3;
    public static final double INTAKE_ARM_MOTOR_KI = 0.000;
    public static final double INTAKE_ARM_MOTOR_KD = 0.00;
    public static final int INTAKE_ARM_POSITION_IN_FULL = 0;
    public static final int INTAKE_ARM_POSITION_IN = 220;
    public static final int INTAKE_ARM_POSITION_OUT = 650;

    public static final double CLIMB6_MOTOR_KF = 0.00;
    public static final double CLIMB6_MOTOR_KP = 5.4;
    public static final double CLIMB6_MOTOR_KI = 0.0150;
    public static final double CLIMB6_MOTOR_KD = 0.0135;
    public static final double CLIMB6_MOTOR_CRUISE = 4000;
    public static final double CLIMB6_MOTOR_ACCELERATION = 8000;
    public static final int CLIMB6_POSITION_IN = 0;
}
