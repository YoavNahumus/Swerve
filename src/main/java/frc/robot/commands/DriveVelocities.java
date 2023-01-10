// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.SwerveConstants;
import frc.robot.subsystems.Chassis;
import frc.robot.utils.General;

public class DriveVelocities extends CommandBase {
    private final Chassis chassis;
    private final XboxController controller;

    public DriveVelocities(Chassis chassis, XboxController controller) {
        this.chassis = chassis;
        this.controller = controller;
        addRequirements(chassis);
    }

    @Override
    public void execute() {
        Translation2d xy = General.getScaledXY(controller, 2);
        double vx = -xy.getY() * SwerveConstants.MAX_DRIVE_SPEED;
        double vy = -xy.getX() * SwerveConstants.MAX_DRIVE_SPEED;
        double omega = General
                .scale(General.deadband(controller.getLeftTriggerAxis() - controller.getRightTriggerAxis()), 2)
                * SwerveConstants.MAX_ANGULAR_SPEED;

        if (vx == 0 && vy == 0 && omega == 0)
            chassis.stop();
        else
            chassis.setVelocities(vx, vy, omega);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        chassis.stop();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}
