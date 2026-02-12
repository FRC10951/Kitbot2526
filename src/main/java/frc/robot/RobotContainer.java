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
import static frc.robot.Constants.FuelConstants.*;

import frc.robot.commands.AutoDrive;
import frc.robot.commands.Drive;
import frc.robot.commands.Eject;
import frc.robot.commands.ExampleAuto;
import frc.robot.commands.Intake;
import frc.robot.commands.LaunchSequence;
import frc.robot.commands.SpinUp;
import frc.robot.subsystems.CANDriveSubsystem;
import frc.robot.subsystems.CANFuelSubsystem;
import frc.robot.subsystems.IoSubsystem;
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
  // private final CANFuelSubsystem fuelSubsystem = new CANFuelSubsystem();
  private final IoSubsystem ioSubsystem = new IoSubsystem();

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

    // autoChooser.addOption(
    // "Autoaim",
    // new AutoDrive(driveSubsystem, 0.5, 0.0).withTimeout(.25)
    // .andThen(new VisionAlign(driveSubsystem, driverController))
    // autoChooser.addOption("Example", new ExampleAuto(driveSubsystem,
    // fuelSubsystem));
    autoChooser.setDefaultOption("Do Nothing", Commands.none());
  }

  public CANDriveSubsystem getDriveSubsystem() {
    return driveSubsystem;
  }

  public IoSubsystem getIoSubsystem() {
    return ioSubsystem;
  }

  private void configureBindings() {
    driveSubsystem.setDefaultCommand(new Drive(driveSubsystem, driverController));
    ioSubsystem.setDefaultCommand(ioSubsystem.commandStop());

    driverController
        .leftBumper()
        .debounce(1)
        .whileTrue(ioSubsystem.comamndPrepare())
        .multiPress(2, 1)
        .whileTrue(ioSubsystem.commandLaunch());

    driverController
        .rightBumper()
        .debounce(1)
        .whileTrue(ioSubsystem.commandIntake())
        .multiPress(2, 1)
        .whileTrue(ioSubsystem.commandEject());

    driverController
        .a()
        .whileTrue(new VisionAlign(driveSubsystem, driverController));

    driverController.b()
        .onTrue(driveSubsystem.runOnce(() -> driveSubsystem.resetHeading()));
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
