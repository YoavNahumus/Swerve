// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.Arrays;

import com.ctre.phoenix.sensors.PigeonIMU;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.SwerveConstants;
import frc.robot.Constants.SwerveModuleConstants;
import frc.robot.commands.templates.InstantCommandInDisable;
import frc.robot.utils.General;
import frc.robot.utils.SwerveModule;

public class Chassis extends SubsystemBase {
    private final Field2d field;
    private final SwerveModule[] modules;
    private final PigeonIMU gyro;
    private final SwerveDrivePoseEstimator poseEstimator;
    private boolean isBreak;

    public Chassis() {
        field = new Field2d();
        gyro = new PigeonIMU(SwerveConstants.GYRO_ID);
        modules = new SwerveModule[] {
                new SwerveModule(SwerveModuleConstants.FRONT_LEFT),
                new SwerveModule(SwerveModuleConstants.FRONT_RIGHT),
                new SwerveModule(SwerveModuleConstants.BACK_LEFT),
                new SwerveModule(SwerveModuleConstants.BACK_RIGHT)
        };
        poseEstimator = new SwerveDrivePoseEstimator(SwerveConstants.KINEMATICS, getGyroRotation(),
                getModulePositions(), new Pose2d(0, 0, getGyroRotation()));
        isBreak = true;
    }

    /**
     * Gets the angle of the robot
     * 
     * @return The angle of the robot, between 0 and 360 degrees
     */
    public double getAngle() {
        return General.normalizeAngle(gyro.getFusedHeading());
    }

    /**
     * Gets the rotation of the robot
     * 
     * @return The rotation of the robot
     */
    public Rotation2d getGyroRotation() {
        return Rotation2d.fromDegrees(getAngle());
    }

    public Rotation2d getRotation() {
        return poseEstimator.getEstimatedPosition().getRotation();
    }

    /**
     * Sets the velocities of the robot
     * 
     * @param vx    The x velocity, in meters per second
     * @param vy    The y velocity, in meters per second
     * @param omega The angular velocity, in radians per second
     */
    public void setVelocities(double vx, double vy, double omega) {
        ChassisSpeeds speeds = ChassisSpeeds.fromFieldRelativeSpeeds(vx, vy, omega, getRotation());
        SwerveModuleState[] states = SwerveConstants.KINEMATICS.toSwerveModuleStates(speeds);
        setModuleStates(states);
    }

    /**
     * Sets the states of the modules
     * 
     * @param states The states of the modules, in order of front left, front right,
     *               back left, back right
     */
    public void setModuleStates(SwerveModuleState[] states) {
        SwerveDriveKinematics.desaturateWheelSpeeds(states, SwerveConstants.MAX_SPEED);
        for (int i = 0; i < 4; i++) {
            states[i] = SwerveModuleState.optimize(states[i], modules[i].getAngleRotation());
            modules[i].setState(states[i]);
        }
    }

    /**
     * Gets the states of the modules
     * 
     * @return The states of the modules, in order of front left, front right, back
     *         left, back right
     */
    public SwerveModuleState[] getModuleStates() {
        SwerveModuleState[] states = new SwerveModuleState[4];
        for (int i = 0; i < 4; i++) {
            states[i] = modules[i].getState();
        }
        return states;
    }

    /**
     * Gets the pose of the robot
     * 
     * @return The pose of the robot
     */
    public Pose2d getPose() {
        return poseEstimator.getEstimatedPosition();
    }

    /**
     * Stops all motors
     */
    public void stop() {
        for (SwerveModule module : modules) {
            module.stopAngleMotor();
            module.stopMoveMotor();
        }
    }

    /**
     * Swaps the neutral mode of the modules, between brake and coast
     */
    public void swapNeutralMode() {
        isBreak = !isBreak;
        for (var module : modules) {
            module.setNeutralMode(isBreak);
        }
    }

    /**
     * Resets the angle of the robot, so the forward of the robot is the same as the
     * forward of the field
     */
    public void resetAngle() {
        gyro.setYaw(0);
        gyro.setFusedHeading(0);
        poseEstimator.resetPosition(getGyroRotation(), getModulePositions(),
                new Pose2d(poseEstimator.getEstimatedPosition().getTranslation(), new Rotation2d()));
    }

    /**
     * Gets the positions of the modules
     * 
     * @return The positions of the modules, in order of front left, front right,
     *         back left, back right
     */
    public SwerveModulePosition[] getModulePositions() {
        return Arrays.stream(modules).map((module) -> module.getPosition()).toArray(SwerveModulePosition[]::new);
    }

    @Override
    public void periodic() {
        poseEstimator.update(getGyroRotation(), getModulePositions());
        field.setRobotPose(getPose());
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        SmartDashboard.putData("Front Left Module", modules[0]);
        SmartDashboard.putData("Front Right Module", modules[1]);
        SmartDashboard.putData("Back Left Module", modules[2]);
        SmartDashboard.putData("Back Right Module", modules[3]);

        SmartDashboard.putData("Field", field);

        builder.addDoubleProperty("Angle", this::getAngle, null);

        SmartDashboard.putData("Change Neutral", new InstantCommandInDisable(this::swapNeutralMode));

        SmartDashboard.putData("Zero Angle", new InstantCommandInDisable(this::resetAngle));

        SmartDashboard.putData("Calibrate Offsets", new InstantCommandInDisable(() -> {
            for (var module : modules) {
                module.calibrateOffset();
            }
        }));
    }
}
