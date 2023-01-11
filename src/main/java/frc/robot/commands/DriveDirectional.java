// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.SwerveConstants;
import frc.robot.subsystems.Chassis;
import frc.robot.utils.General;
import frc.robot.utils.General.ControllerSide;

/**
 * Drives the robot using the left stick for velocity and the right stick for heading.
 */
public class DriveDirectional extends CommandBase {
    private final Chassis chassis;
    private final XboxController controller;

    /**
     * Creates a new DriveDirectional.
     * 
     * @param chassis    The chassis to drive
     * @param controller The controller to get input from (left stick is used for velocity, right stick for heading)
     */
    public DriveDirectional(Chassis chassis, XboxController controller) {
        this.chassis = chassis;
        this.controller = controller;
        addRequirements(chassis);
    }

    @Override
    public void execute() {
        Translation2d xy = General.getScaledStick(controller, ControllerSide.LEFT, 2);
        double vx = xy.getY() * SwerveConstants.MAX_DRIVE_SPEED;
        double vy = -xy.getX() * SwerveConstants.MAX_DRIVE_SPEED;
        Rotation2d angle = General.getStickRotation(controller, ControllerSide.RIGHT);

        if (vx == 0 && vy == 0 && angle == null)
            chassis.stop();
        else
            chassis.setAngleAndVelocity(vx, vy, angle.getRadians());
    }

    @Override
    public void end(boolean interrupted) {
        chassis.stop();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
