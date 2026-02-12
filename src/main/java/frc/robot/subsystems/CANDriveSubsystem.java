// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.Constants.DriveConstants.*;

import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import com.revrobotics.RelativeEncoder;



public class CANDriveSubsystem extends SubsystemBase {
  private final SparkMax leftLeader;
  private final SparkMax leftFollower;
  private final SparkMax rightLeader;
  private final SparkMax rightFollower;
 
  private final DifferentialDrive drive;

  // Encoders for tracking wheel rotations
  private final RelativeEncoder leftEncoder;
  private final RelativeEncoder rightEncoder;

  // Gyroscope for tracking robot rotation
  private final ADXRS450_Gyro gyro;

  // Odometry and pose estimation
  private final DifferentialDriveKinematics kinematics;
  private final DifferentialDrivePoseEstimator poseEstimator;

  // Field visualization
  private final Field2d field2d = new Field2d();

  // Track distance per pulse for encoders (adjust based on your robot)
  // For standard Kitbot with 6-inch wheels and default gearing:
  // Wheel circumference = π * diameter = π * 0.1524 meters ≈ 0.478 meters
  // NEO built-in encoder: 42 counts per revolution
  private static final double WHEEL_DIAMETER_METERS = 0.1524; // 6 inches
  private static final double TRACK_WIDTH_METERS = 0.6; // Distance between left/right wheels (adjust for your robot)


  public CANDriveSubsystem() {
    // create brushed motors for drive
    leftLeader = new SparkMax(LEFT_LEADER_ID, MotorType.kBrushed);
    leftFollower = new SparkMax(LEFT_FOLLOWER_ID, MotorType.kBrushed);
    rightLeader = new SparkMax(RIGHT_LEADER_ID, MotorType.kBrushed);
    rightFollower = new SparkMax(RIGHT_FOLLOWER_ID, MotorType.kBrushed);

    // set up differential drive class
    drive = new DifferentialDrive(leftLeader, rightLeader);

    // Set can timeout. Because this project only sets parameters once on
    // construction, the timeout can be long without blocking robot operation. Code
    // which sets or gets parameters during operation may need a shorter timeout.
    leftLeader.setCANTimeout(250);
    rightLeader.setCANTimeout(250);
    leftFollower.setCANTimeout(250);
    rightFollower.setCANTimeout(250);

    // Create separate configurations to apply to motors. Voltage compensation
    // helps the robot perform more similarly on different battery voltages (at the
    // cost of a little bit of top speed on a fully charged battery). The current
    // limit helps prevent tripping breakers.

    // Leader configs
    SparkMaxConfig leftLeaderConfig = new SparkMaxConfig();
    leftLeaderConfig.voltageCompensation(12);
    leftLeaderConfig.smartCurrentLimit(DRIVE_MOTOR_CURRENT_LIMIT);
    // Set left side inverted so that positive values drive both sides forward
    leftLeaderConfig.inverted(true);
    leftLeader.configure(leftLeaderConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    SparkMaxConfig rightLeaderConfig = new SparkMaxConfig();
    rightLeaderConfig.voltageCompensation(12);
    rightLeaderConfig.smartCurrentLimit(DRIVE_MOTOR_CURRENT_LIMIT);
    // Right side not inverted
    rightLeader.configure(rightLeaderConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    // Follower configs: explicitly follow their respective leaders
    SparkMaxConfig leftFollowerConfig = new SparkMaxConfig();
    leftFollowerConfig.follow(leftLeader);
    leftFollower.configure(leftFollowerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    SparkMaxConfig rightFollowerConfig = new SparkMaxConfig();
    rightFollowerConfig.follow(rightLeader);
    rightFollower.configure(rightFollowerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    // Get built-in encoders from NEO motors
    leftEncoder = leftLeader.getEncoder();
    rightEncoder = rightLeader.getEncoder();

    // NOTE: Some REV library versions do not expose position/velocity
    // conversion-factor setters on RelativeEncoder. For maximum compatibility,
    // we leave the encoders in their native units (rotations) and apply any
    // necessary scaling at the point of use instead.

    // Initialize gyroscope
    gyro = new ADXRS450_Gyro();
    gyro.calibrate();

    // Initialize kinematics with track width
    kinematics = new DifferentialDriveKinematics(TRACK_WIDTH_METERS);

    // Initialize pose estimator (combines odometry + vision)
    poseEstimator = new DifferentialDrivePoseEstimator(
        kinematics,
        gyro.getRotation2d(),
        leftEncoder.getPosition(),
        rightEncoder.getPosition(),
        new Pose2d() // Start at origin
    );

    // Put field visualization on dashboard
    SmartDashboard.putData("Field", field2d);

  }

  @Override
  public void periodic() {
      // Update pose estimator with encoder and gyro data
      poseEstimator.update(
          gyro.getRotation2d(),
          leftEncoder.getPosition(),
          rightEncoder.getPosition()
      );

      // Update field visualization
      field2d.setRobotPose(poseEstimator.getEstimatedPosition());
  }


  public void driveArcade(double xSpeed, double zRotation) {
    drive.arcadeDrive(xSpeed, zRotation);
  }

  /**
   * Adds a vision measurement to the pose estimator.
   * This corrects the odometry estimate using AprilTag detections.
   *
   * @param visionPose The pose measured by the Limelight
   * @param timestampSeconds The timestamp of the measurement
   */
  public void addVisionMeasurement(Pose2d visionPose, double timestampSeconds) {
      poseEstimator.addVisionMeasurement(visionPose, timestampSeconds);
  }

  /**
   * Gets the current estimated pose of the robot.
   *
   * @return The robot's pose on the field
   */
  public Pose2d getPose() {
      return poseEstimator.getEstimatedPosition();
  }

  /**
   * Resets the pose estimator to a specific pose.
   *
   * @param pose The new pose to reset to
   */
  public void resetPose(Pose2d pose) {
      poseEstimator.resetPosition(
          gyro.getRotation2d(),
          leftEncoder.getPosition(),
          rightEncoder.getPosition(),
          pose
      );
  }

  /**
   * Gets the angular velocity of the robot in rotations per second.
   *
   * @return Angular velocity in rotations/second
   */
  public double getAngularVelocity() {
      // Gyro rate is in degrees/second, convert to rotations/second
      return Math.abs(gyro.getRate()) / 360.0;
  }

  /**
   * Gets the current gyro heading.
   *
   * @return Gyro rotation as Rotation2d
   */
  public Rotation2d getHeading() {
      return gyro.getRotation2d();
  }

  /**
   * Resets the gyro heading to zero.
   */
  public void resetHeading() {
      gyro.reset();
  }


}
