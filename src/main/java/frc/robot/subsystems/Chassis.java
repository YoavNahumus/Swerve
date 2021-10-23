// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.sensors.PigeonIMU;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableBuilder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.commands.Drive;
import frc.robot.utils.SwerveUnit;

public class Chassis extends SubsystemBase {
  /** Creates a new Chassis. */
  private final SwerveUnit topLeft;
  private final SwerveUnit topRight;
  private final SwerveUnit backLeft;
  private final SwerveUnit backRight;

  private final PigeonIMU gyro;

  private static final double L = Constants.ch_trackLength, W = Constants.ch_trackWidth, R = Constants.ch_trackDiagonal;

  public Chassis(XboxController controller) {
    topLeft = new SwerveUnit(Constants.ch_dirMotorTL, Constants.ch_moveMotorTL);
    topRight = new SwerveUnit(Constants.ch_dirMotorTR, Constants.ch_moveMotorTR);
    backLeft = new SwerveUnit(Constants.ch_dirMotorBL, Constants.ch_moveMotorBL);
    backRight = new SwerveUnit(Constants.ch_dirMotorBR, Constants.ch_moveMotorBR);
    
    topLeft.setMoveInverted(false);
    topRight.setMoveInverted(false);
    backLeft.setMoveInverted(false);
    backRight.setMoveInverted(false);

    topLeft.setDirInverted(false);
    topRight.setDirInverted(false);
    backLeft.setDirInverted(false);
    backRight.setDirInverted(false);

    topLeft.setMoveSensorPhase(false);
    topRight.setMoveSensorPhase(false);
    backLeft.setMoveSensorPhase(false);
    backRight.setMoveSensorPhase(false);

    topLeft.setDirSensorPhase(false);
    topRight.setDirSensorPhase(false);
    backLeft.setDirSensorPhase(false);
    backRight.setDirSensorPhase(false);

    gyro = new PigeonIMU(Constants.ch_gyroPort);

    setDefaultCommand(new Drive(this, controller));
  }

  public double getAngle(){
    return gyro.getFusedHeading();
  }

  public double getRadianAngle(){
    return Math.toRadians(getAngle());
  }

  public void setAllAnglesSmart(double angleTL, double angleTR, double angleBL, double angleBR){
    topLeft.setAngleSmart(angleTL);
    topRight.setAngleSmart(angleTR);
    backLeft.setAngleSmart(angleBL);
    backRight.setAngleSmart(angleBR);
  }

  public void setAllVelocities(double velTL, double velTR, double velBL, double velBR){
    topLeft.setVelocity(velTL);
    topRight.setVelocity(velTR);
    backLeft.setVelocity(velBL);
    backRight.setVelocity(velBR);
  }

  public void setAllBrake(boolean isBrake){
    topLeft.setBrakeMode(isBrake);
    backLeft.setBrakeMode(isBrake);
    topRight.setBrakeMode(isBrake);
    backRight.setBrakeMode(isBrake);
  }

  public void stopMotors(){
    topLeft.stopMotors();
    backLeft.stopMotors();
    topRight.stopMotors();
    backRight.stopMotors();
  }

  /**
   * swerve drive movement
   * @param forward the forward speed between (-1,1)
   * @param right the right speed between (-1,1) 
   * @param rotation the rotation clockwise between (-1,1)
   */
  public void setDirection_Rotation(double forward, double right, double rotation){
    double angle = -getRadianAngle();

    double temp = forward * Math.cos(angle) + right * Math.sin(angle);
    right = -forward * Math.sin(angle) + right * Math.cos(angle);
    forward = temp;
    
    double A, B, C, D;
    A = right - rotation * (L / R);
    B = right + rotation * (L / R);
    C = forward - rotation * (W / R);
    D = forward + rotation * (W / R);

    double wheelSpeedFR, wheelSpeedFL, wheelSpeedBL, wheelSpeedBR;

    wheelSpeedFR = Math.sqrt(B * B + C * C);
    wheelSpeedFL = Math.sqrt(B * B + D * D);
    wheelSpeedBL = Math.sqrt(A * A + D * D);
    wheelSpeedBR = Math.sqrt(A * A + C * C);

    double wheelAngleFR, wheelAngleFL, wheelAngleBL, wheelAngleBR;

    wheelAngleFR = -(Math.atan2(B, C) * 180. / Math.PI) + 180;
    wheelAngleFL = -(Math.atan2(B, D) * 180. / Math.PI) + 180;
    wheelAngleBL = -(Math.atan2(A, D) * 180. / Math.PI) + 180;
    wheelAngleBR = -(Math.atan2(A, C) * 180. / Math.PI) + 180;

    double max = Math.max(Math.max(wheelSpeedFR, wheelSpeedFL), Math.max(wheelSpeedBL, wheelSpeedBR));

    if (max > 1){
      wheelSpeedFR /= max;
      wheelSpeedFL /= max;
      wheelSpeedBR /= max;
      wheelSpeedBL /= max;
    }

    setAllAnglesSmart(wheelAngleFL, wheelAngleFR, wheelAngleBL, wheelAngleBR);
    setAllVelocities(wheelSpeedFL, wheelSpeedFR, wheelSpeedBL, wheelSpeedBR);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    builder.addDoubleProperty("Angle", this::getAngle, null);
    topLeft.initSendable(builder);
    topRight.initSendable(builder);
    backLeft.initSendable(builder);
    backRight.initSendable(builder);
  }
}
