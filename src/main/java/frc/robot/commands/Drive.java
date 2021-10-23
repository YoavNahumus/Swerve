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
  private final Chassis chassis;
  private final XboxController controller;

  private static final double movementDeadBand = 0.15;
  private static final double rotationDeadBand = 0.1;

  public Drive(Chassis chassis, XboxController controller) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.chassis = chassis;
    this.controller = controller;
    addRequirements(chassis);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double movementX = deadBand(controller.getX(Hand.kLeft), movementDeadBand);
    double movementY = deadBand(-controller.getY(Hand.kLeft), movementDeadBand);
    double rotation = deadBand(controller.getTriggerAxis(Hand.kRight) - controller.getTriggerAxis(Hand.kLeft), rotationDeadBand);

    if (movementX == 0 && movementY == 0 && rotation == 0){
      chassis.stopMotors();
      chassis.setAllBrake(true);
    }
    else {
      chassis.setAllBrake(false);
      chassis.setDirection_Rotation(movementY, movementX, rotation);
    }
  }

  /**
   * dead-bands the values
   * @param value the value to be dead-banded
   * @param min the minimum value acceptable, in absolute
   * @return 0 if the value is too little, value otherwise
   */
  public double deadBand(double value, double min){
    if (value < min && value > -min){
      return 0;
    }
    return value;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    chassis.stopMotors();
    chassis.setAllBrake(false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
