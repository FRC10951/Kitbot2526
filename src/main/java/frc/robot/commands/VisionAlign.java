// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.LimelightHelpers;
import frc.robot.subsystems.CANDriveSubsystem;
import static frc.robot.Constants.OperatorConstants.*;
import static frc.robot.Constants.VisionConstants.*;

/**
 * Vision-assisted driving command.
 * Driver controls forward/backward with left stick Y-axis,
 * but rotation automatically aligns to the detected AprilTag.
 */
public class VisionAlign extends Command {
    private final CANDriveSubsystem driveSubsystem;
    private final CommandXboxController controller;

    // Proportional gain for rotation correction (tune as needed)
    private static final double ROTATION_KP = VISION_ROTATION_KP;

    public VisionAlign(CANDriveSubsystem driveSystem, CommandXboxController driverController) {
        addRequirements(driveSystem);
        this.driveSubsystem = driveSystem;
        this.controller = driverController;
    }

    @Override
    public void initialize() {
        // No initialization needed
    }

    @Override
    public void execute() {
        // Get forward speed from driver's left Y-axis
        double forwardSpeed = -controller.getLeftY() * DRIVE_SCALING;

        // Get horizontal angle error from Limelight (TX)
        double tx = LimelightHelpers.getTX("limelight");

        // Calculate rotation correction (proportional control)
        // Negative TX = target is left → rotate left (negative rotation)
        // Positive TX = target is right → rotate right (positive rotation)
        double rotationCorrection = -tx * ROTATION_KP;

        // Apply limits to rotation correction
        rotationCorrection = Math.max(-ROTATION_SCALING, Math.min(ROTATION_SCALING, rotationCorrection));

        // Drive with manual forward/backward, auto rotation
        driveSubsystem.driveArcade(forwardSpeed, rotationCorrection);
    }

    @Override
    public void end(boolean interrupted) {
        // Stop the robot when command ends
        driveSubsystem.driveArcade(0, 0);
    }

    @Override
    public boolean isFinished() {
        // Command runs until button is released
        return false;
    }
}
