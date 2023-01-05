// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
package frc.robot.utils;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.sensors.CANCoder;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.util.sendable.SendableRegistry;
import frc.robot.Constants.SwerveModuleConstants;

public class SwerveModule implements Sendable {
    private double angleOffset;
    private final TalonFX moveMotor, angleMotor;
    private final CANCoder absoluteEncoder;
    private final SimpleMotorFeedforward feedforward;

    public SwerveModule(SwerveModuleConstants constants) {
        angleOffset = constants.angleOffset;
        moveMotor = new TalonFX(constants.moveMotorID);
        angleMotor = new TalonFX(constants.angleMotorID);
        absoluteEncoder = new CANCoder(constants.absoluteEncoderID);
        feedforward = new SimpleMotorFeedforward(constants.kS, constants.kV);
        configureDevices();
    }

    /**
     * Configures the devices to their default values
     */
    private void configureDevices() {
        moveMotor.configFactoryDefault();
        angleMotor.configFactoryDefault();
        absoluteEncoder.configFactoryDefault();

        moveMotor.config_kP(0, SwerveModuleConstants.VELOCITY_KP);

        angleMotor.config_kP(0, SwerveModuleConstants.ANGLE_KP);
        angleMotor.config_kI(0, SwerveModuleConstants.ANGLE_KI);
        angleMotor.setNeutralMode(NeutralMode.Brake);
        moveMotor.setNeutralMode(NeutralMode.Brake);
    }

    /**
     * Gets the angle of the module, accounting for the offset
     * @return The angle of the module, between 0 and 360 degrees
     */
    public double getAngle() {
        return General.normalizeAngle(absoluteEncoder.getAbsolutePosition() - angleOffset);
    }

    /**
     * Gets the angle of the module as a Rotation2d
     * @return The angle of the module as a Rotation2d
     */
    public Rotation2d getAngleRotation() {
        return Rotation2d.fromDegrees(getAngle());
    }

    /**
     * Gets the velocity of the module
     * @return The velocity of the module, in meters per second
     */
    public double getVelocity() {
        return moveMotor.getSelectedSensorVelocity() / SwerveModuleConstants.PULSE_PER_METER * 10;
    }

    /**
     * Sets the velocity of the module
     * @param velocity The velocity to set the module to, in meters per second
     */
    public void setVelocity(double velocity) {
        moveMotor.set(ControlMode.Velocity, velocity * SwerveModuleConstants.PULSE_PER_METER / 10,
                DemandType.ArbitraryFeedForward, feedforward.calculate(velocity));
    }

    /**
     * Calculates the target angle for the module
     * @param targetAngle The target angle, in degrees
     * @return The target angle, in encoder pulses
     */
    public double calculateTarget(double targetAngle) {
        double difference = General.getAngleDifference(getAngle(), targetAngle);
        return angleMotor.getSelectedSensorPosition() + (difference * SwerveModuleConstants.PULSE_PER_DEGREE);
    }

    /**
     * Sets the angle of the module
     * @param angle The angle to set the module to, in degrees
     */
    public void setAngle(double angle) {
        angleMotor.set(ControlMode.Position, calculateTarget(angle));
    }

    /**
     * Stops the angle motor
     */
    public void stopAngleMotor() {
        angleMotor.set(ControlMode.PercentOutput, 0);
    }

    /**
     * Stops the move motor
     */
    public void stopMoveMotor() {
        moveMotor.set(ControlMode.PercentOutput, 0);
    }

    /**
     * Gets the state of the module
     * @return The state of the module
     */
    public SwerveModuleState getState() {
        return new SwerveModuleState(getVelocity(), getAngleRotation());
    }

    /**
     * Sets the state of the module
     * @param state The state to set the module to
     */
    public void setState(SwerveModuleState state) {
        setVelocity(state.speedMetersPerSecond);
        setAngle(state.angle.getDegrees());
    }

    /**
     * Sets the power of the velocity motor
     * @param power The power to set the velocity motor to
     */
    public void setVelocityPower(double power) {
        moveMotor.set(ControlMode.PercentOutput, power);
    }

    /**
     * Sets the neutral mode of the module
     * @param isBreak Whether the module should be in brake mode or in coast mode
     */
    public void setNeutralMode(boolean isBreak) {
        angleMotor.setNeutralMode(isBreak ? NeutralMode.Brake : NeutralMode.Coast);
        moveMotor.setNeutralMode(isBreak ? NeutralMode.Brake : NeutralMode.Coast);
    }

    /**
     * Sets the offset of the module to the current angle
     */
    public void calibrateOffset() {
        angleOffset = absoluteEncoder.getAbsolutePosition();
        System.out.println("Offset " + SendableRegistry.getName(this) + ": " + angleOffset);
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.addDoubleProperty("Velocity", this::getVelocity, null);
        builder.addDoubleProperty("Angle", this::getAngle, null);
        builder.addDoubleProperty("Angle Error", angleMotor::getClosedLoopError, null);
        builder.addDoubleProperty("Velocity Error", moveMotor::getClosedLoopError, null);
        builder.addDoubleProperty("Angle Offset", () -> angleOffset, null);
    }
}
