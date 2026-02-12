// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean constants. This class should not be used for any other
 * purpose. All constants should be declared globally (i.e. public static). Do
 * not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static final class DriveConstants {
    // Motor controller IDs for drivetrain motors
    public static final int LEFT_LEADER_ID = 11;
    public static final int LEFT_FOLLOWER_ID = 8;
    public static final int RIGHT_LEADER_ID = 10;
    public static final int RIGHT_FOLLOWER_ID = 7;

    // Current limit for drivetrain motors. 60A is a reasonable maximum to reduce
    // likelihood of tripping breakers or damaging CIM motors
    public static final int DRIVE_MOTOR_CURRENT_LIMIT = 60;
  }

  public static final class IoConstants {
    public static final int IO_MOTOR_ID = 9;
    public static final int LOADER_MOTOR_ID = 19;

    public static final int IO_MOTOR_CURRENT_LIMIT = 60;
    public static final int LOADER_MOTOR_CURRENT_LIMIT = 60;

    public static final double INTAKING_IO_VOLTAGE = -12;
    public static final double INTAKING_LOADER_VOLTAGE = 10;

    public static final double PREPARING_IO_VOLTAGE = -6;
    public static final double PREPARING_LOADER_VOLTAGE = 0;

    public static final double LAUNCHING_IO_VOLTAGE = 12;
    public static final double LAUNCHING_LOADER_VOLTAGE = 10;
  }

  // TODO: delete this once CANFuelSubsytem is removed
  public static final class FuelConstants {
    // Motor controller IDs for Fuel Mechanism motors
    public static final int FEEDER_MOTOR_ID = 9;
    public static final int INTAKE_LAUNCHER_MOTOR_ID = 19;

    // Current limit and nominal voltage for fuel mechanism motors.
    public static final int FEEDER_MOTOR_CURRENT_LIMIT = 60;
    public static final int LAUNCHER_MOTOR_CURRENT_LIMIT = 60;

    // Voltage values for various fuel operations. These values may need to be tuned
    // based on exact robot construction.
    // See the Software Guide for tuning information
    public static final double INTAKING_FEEDER_VOLTAGE = -12;
    public static final double INTAKING_INTAKE_VOLTAGE = 10;
    public static final double LAUNCHING_FEEDER_VOLTAGE = 9;
    public static final double LAUNCHING_LAUNCHER_VOLTAGE = 10.6;
    public static final double SPIN_UP_FEEDER_VOLTAGE = -6;
    public static final double SPIN_UP_SECONDS = 1;
  }

  public static final class OperatorConstants {
    // Port constants for driver and operator controllers. These should match the
    // values in the Joystick tab of the Driver Station software
    public static final int DRIVER_CONTROLLER_PORT = 0;
    public static final int OPERATOR_CONTROLLER_PORT = 1;

    // This value is multiplied by the joystick value when rotating the robot to
    // help avoid turning too fast and beign difficult to control
    public static final double DRIVE_SCALING = .7;
    public static final double ROTATION_SCALING = .8;
  }

  public static final class VisionConstants {
    // Limelight camera name (must match network tables configuration)
    public static final String LIMELIGHT_NAME = "limelight";

    // Maximum rotations per second threshold for accepting vision measurements
    // Robot must be rotating slower than this to trust vision data
    public static final double MAX_ANGULAR_VELOCITY_RPS = 2.0;

    // Proportional gain for AprilTag rotation alignment
    // Higher = more aggressive rotation, Lower = smoother but slower
    // Recommended range: 0.02 - 0.05
    public static final double VISION_ROTATION_KP = 0.03;
  }

}
