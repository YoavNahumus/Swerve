// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.SwerveConstants;
import frc.robot.subsystems.Chassis;
import frc.robot.utils.Utils;
import frc.robot.utils.Utils.ControllerSide;

/**
 * Drives the robot using the left stick for velocity and the triggers for
 * rotation.
 */
public class Drive extends CommandBase {
    private final Chassis chassis;
    private final XboxController controller;
    private double scaleVelocity = 2;
    private double scaleRotation = 2;

    /**
     * Creates a new DriveVelocities.
     * 
     * @param chassis    The chassis to drive
     * @param controller The controller to get input from (left stick is used for
     *                   velocity, triggers or right stick for rotation)
     */
    public Drive(Chassis chassis, XboxController controller) {
        this.chassis = chassis;
        this.controller = controller;
        addRequirements(chassis);
    }

    @Override
    public void execute() {
        boolean red = Utils.isRedAlliance();
        Translation2d xy = Utils.getScaledStick(controller, ControllerSide.LEFT, scaleVelocity)
                .times(red ? -1 : 1);
        double vx = xy.getY() * SwerveConstants.MAX_DRIVE_SPEED;
        double vy = -xy.getX() * SwerveConstants.MAX_DRIVE_SPEED;
        double omega = Utils.getScaledTriggerDiff(controller, ControllerSide.LEFT, scaleRotation)
                * SwerveConstants.MAX_ANGULAR_SPEED;
        Rotation2d angle = Utils.getStickRotation(controller, ControllerSide.RIGHT);

        if (vx == 0 && vy == 0 && omega == 0 && angle == null)
            chassis.stop();
        else if (angle == null)
            chassis.setVelocities(vx, vy, omega);
        else
            chassis.setAngleAndVelocity(vx, vy, angle.getRadians() + Math.PI / 2 * (red ? 1 : -1));
    }

    @Override
    public void end(boolean interrupted) {
        chassis.stop();
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.addDoubleProperty("Velocity Scale", () -> scaleVelocity, (s) -> scaleVelocity = s);
        builder.addDoubleProperty("Rotation Scale", () -> scaleRotation, (s) -> scaleRotation = s);
    }
}
