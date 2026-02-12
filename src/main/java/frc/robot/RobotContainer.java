// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import static frc.robot.Constants.OperatorConstants.*;
<<<<<<< HEAD
import static frc.robot.Constants.FuelConstants.*;
=======

import frc.robot.commands.AutoDrive;
>>>>>>> 44537bc (feat: thursday)
import frc.robot.commands.Drive;
import frc.robot.commands.Eject;
import frc.robot.commands.ExampleAuto;
import frc.robot.commands.Intake;
import frc.robot.commands.LaunchSequence;
import frc.robot.commands.SpinUp;
import frc.robot.subsystems.CANDriveSubsystem;
import frc.robot.subsystems.CANFuelSubsystem;

import frc.robot.LimelightHelpers;
import frc.robot.commands.VisionAlign;


/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link Robot} periodic methods (other than the
 * scheduler calls). Instead, the structure of the robot (including subsystems,
 * commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems
  private final CANDriveSubsystem driveSubsystem = new CANDriveSubsystem();
  private final CANFuelSubsystem fuelSubsystem = new CANFuelSubsystem();

  // The driver's controller
  private final CommandXboxController driverController = new CommandXboxController(
      DRIVER_CONTROLLER_PORT);

  // The operator's controller
  private final CommandXboxController operatorController = new CommandXboxController(
      OPERATOR_CONTROLLER_PORT);

  // The autonomous chooser
  private final SendableChooser<Command> autoChooser = new SendableChooser<>();

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    configureBindings();

    // Set the options to show up in the Dashboard for selecting auto modes. If you
    // add additional auto modes you can add additional lines here with
    // autoChooser.addOption 
    autoChooser.addOption("Leftmost", 
    new AutoDrive(driveSubsystem,0.5,  0.0).withTimeout(.25).andThen(new SpinUp(fuelSubsystem))
    );
    autoChooser.addOption("Center", new ExampleAuto(driveSubsystem, fuelSubsystem));
    autoChooser.addOption("Rightmost", new ExampleAuto(driveSubsystem, fuelSubsystem));
    autoChooser.addOption("Example", new ExampleAuto(driveSubsystem, fuelSubsystem));
    autoChooser.setDefaultOption("Do Nothing", Commands.none());
  }

  /**
   * Returns the drive subsystem for vision integration.
   *
   * @return The CANDriveSubsystem instance
   */
  public CANDriveSubsystem getDriveSubsystem() {
      return driveSubsystem;
  }


  /**
   * Use this method to define your trigger->command mappings. Triggers can be
   * created via the {@link Trigger#Trigger(java.util.function.BooleanSupplier)}
   * constructor with an arbitrary predicate, or via the named factories in
   * {@link edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses
   * for {@link CommandXboxController Xbox}/
   * {@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller PS4}
   * controllers or
   * {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {

    // While the left bumper on operator controller is held, intake Fuel
    operatorController.leftBumper().whileTrue(new Intake(fuelSubsystem));
    // X button: run CAN ID 9 (feeder) at shooting speed while held
    operatorController.x().whileTrue(
        fuelSubsystem.run(() -> fuelSubsystem.setFeederRoller(LAUNCHING_FEEDER_VOLTAGE))
            .finallyDo(interrupted -> fuelSubsystem.setFeederRoller(0)));
    // While the right bumper on operator controller is held, spin up for 1
    // second, then launch fuel. When the button is released, stop.
    operatorController.rightBumper().whileTrue(new LaunchSequence(fuelSubsystem));
    // Y button: run CAN ID 19 (intake/launcher) at shooting speed while held
    operatorController.y().whileTrue(
        fuelSubsystem.run(() -> fuelSubsystem.setIntakeLauncherRoller(LAUNCHING_LAUNCHER_VOLTAGE))
            .finallyDo(interrupted -> fuelSubsystem.setIntakeLauncherRoller(0)));
    // While the A button is held on the operator controller, eject fuel back out
    // the intake
    operatorController.a().whileTrue(new Eject(fuelSubsystem));

    // Set the default command for the drive subsystem to the command provided by
    // factory with the values provided by the joystick axes on the driver
    // controller. The Y axis of the controller is inverted so that pushing the
    // stick away from you (a negative value) drives the robot forwards (a positive
    // value)
    driveSubsystem.setDefaultCommand(new Drive(driveSubsystem, driverController));

    fuelSubsystem.setDefaultCommand(fuelSubsystem.run(() -> fuelSubsystem.stop()));

    // DRIVER CONTROLS - Vision-assisted rotation
    // Hold LEFT BUMPER on driver controller: Auto-rotate toward AprilTag
    driverController.leftBumper().whileTrue(
        new VisionAlign(driveSubsystem, driverController)
    );

    // Press RIGHT BUMPER on driver controller: Reset gyro heading to 0°
    driverController.rightBumper().onTrue(
        driveSubsystem.runOnce(() -> driveSubsystem.resetHeading())
    );

  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return autoChooser.getSelected();
  }
}
