// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Chassis;

public class Drive extends CommandBase {
  /** Creates a new Drive. */
  private Chassis chassis;
  private XboxController controller;
  public Drive(Chassis chassis, XboxController controller) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.chassis = chassis;
    this.controller = controller;
    addRequirements(chassis);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double movementX = controller.getX(Hand.kLeft);
    double movementY = -controller.getY(Hand.kRight);
    
    double rotation = controller.getTriggerAxis(Hand.kRight) - controller.getTriggerAxis(Hand.kLeft);

    chassis.setDirection_Rotation(movementY, movementX, rotation);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    chassis.setDirection_Rotation(0, 0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
