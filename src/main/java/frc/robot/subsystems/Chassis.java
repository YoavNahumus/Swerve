// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.Arrays;
import java.util.HashMap;
import java.util.Map;

import com.ctre.phoenix.sensors.PigeonIMU;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPoint;
import com.pathplanner.lib.commands.FollowPathWithEvents;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.SwerveConstants;
import frc.robot.Constants.SwerveModuleConstants;
import frc.robot.utils.SwerveModule;
import frc.robot.utils.Utils;
import frc.robot.utils.VisionUtils;

/**
 * The subsystem that controls the robot's swerve chassis
 */
public class Chassis extends SubsystemBase {
    private final Field2d field;
    private final SwerveModule[] modules;
    private final PigeonIMU gyro;
    private final SwerveDrivePoseEstimator poseEstimator;
    private final PIDController angleController;
    private final double startRoll, startPitch;
    private boolean isBreak;

    /**
     * Creates a new Chassis.
     */
    public Chassis() {
        field = new Field2d();
        gyro = new PigeonIMU(SwerveConstants.GYRO_ID);
        modules = new SwerveModule[] {
                new SwerveModule(SwerveModuleConstants.FRONT_LEFT),
                new SwerveModule(SwerveModuleConstants.FRONT_RIGHT),
                new SwerveModule(SwerveModuleConstants.BACK_LEFT),
                new SwerveModule(SwerveModuleConstants.BACK_RIGHT)
        };
        angleController = new PIDController(SwerveConstants.TELEOP_ROTATION_KP,
                SwerveConstants.TELEOP_ROTATION_KI, 0);
        angleController.enableContinuousInput(0, 2 * Math.PI);
        angleController.setTolerance(SwerveConstants.ANGLE_TOLERANCE);
        poseEstimator = new SwerveDrivePoseEstimator(SwerveConstants.KINEMATICS, getGyroRotation(),
                getModulePositions(), new Pose2d(0, 0, getGyroRotation()));
        isBreak = true;

        SmartDashboard.putData(this);

        startPitch = gyro.getPitch();
        startRoll = gyro.getRoll();
    }

    /**
     * Gets the angle of the robot
     * 
     * @return The angle of the robot, between 0 and 360 degrees
     */
    public double getAngle() {
        return Utils.normalizeDegrees(gyro.getFusedHeading());
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
     * Sets the velocities and the angle of the robot
     * 
     * @param vx    The x velocity, in meters per second
     * @param vy    The y velocity, in meters per second
     * @param angle The angle of the robot, in radians
     */
    public void setAngleAndVelocity(double vx, double vy, double angle) {
        angleController.setSetpoint(Utils.normalizeRadians(angle));
        double omega = 0;
        if (!angleController.atSetpoint())
            omega = angleController.calculate(Utils.normalizeRadians(getRotation().getRadians()));
        setVelocities(vx, vy, omega);
    }

    /**
     * Sets the states of the modules
     * 
     * @param states The states of the modules, in order of front left, front right,
     *               back left, back right
     */
    private void setModuleStates(SwerveModuleState[] states) {
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
    private SwerveModuleState[] getModuleStates() {
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
    private void resetAngle() {
        gyro.setYaw(0);
        gyro.setFusedHeading(0);
        while (Math.abs(gyro.getFusedHeading()) > 0.1)
            ;
        poseEstimator.resetPosition(getGyroRotation(), getModulePositions(),
                new Pose2d(poseEstimator.getEstimatedPosition().getTranslation(), new Rotation2d()));
    }

    /**
     * Gets the positions of the modules
     * 
     * @return The positions of the modules, in order of front left, front right,
     *         back left, back right
     */
    private SwerveModulePosition[] getModulePositions() {
        return Arrays.stream(modules).map((module) -> module.getPosition()).toArray(SwerveModulePosition[]::new);
    }

    /**
     * Resets the pose of the robot
     * 
     * @param pose The pose to reset to
     */
    private void resetPose(Pose2d pose) {
        poseEstimator.resetPosition(getGyroRotation(), getModulePositions(), pose);
    }

    /**
     * Gets the velocity of the robot
     * 
     * @return The velocity of the robot, in meters per second
     */
    public Translation2d getVelocity() {
        ChassisSpeeds speeds = SwerveConstants.KINEMATICS.toChassisSpeeds(getModuleStates());
        return new Translation2d(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond);
    }

    /**
     * Creates a path following command, executing events along the way
     * 
     * @param trajectory The trajectory to follow
     * @param events     The events to execute
     * @param resetPose  Whether to reset the pose of the robot at the start of the
     *                   command
     * @return The path following command
     */
    public Command createPathFollowingCommand(PathPlannerTrajectory trajectory, Map<String, Command> events,
            boolean resetPose) {
        var command = new SequentialCommandGroup(
                new InstantCommand(() -> {
                    if (resetPose)
                        resetPose(trajectory.getInitialPose());
                }),
                new PPSwerveControllerCommand(
                        trajectory,
                        this::getPose,
                        SwerveConstants.KINEMATICS,
                        new PIDController(SwerveConstants.AUTO_TRANSLATION_KP, SwerveConstants.AUTO_TRANSLATION_KI, 0),
                        new PIDController(SwerveConstants.AUTO_TRANSLATION_KP, SwerveConstants.AUTO_TRANSLATION_KI, 0),
                        new PIDController(SwerveConstants.AUTO_ROTATION_KP, SwerveConstants.AUTO_ROTATION_KI, 0),
                        this::setModuleStates,
                        this));

        return new FollowPathWithEvents(command, trajectory.getMarkers(), events);
    }

    /**
     * Creates a path following command
     * 
     * @param path   The path to follow
     * @param events The events to run on the markers in the path
     * @return the path following command
     */
    public Command createPathFollowingCommand(String path, Map<String, Command> events) {
        var trajectory = PathPlanner.loadPath(path, SwerveConstants.PATH_CONSTRAINTS);
        return createPathFollowingCommand(trajectory, events, false);
    }

    /**
     * Creates a path following command
     * 
     * @param path      The path to follow
     * @param events    The events to run on the markers in the path
     * @param resetPose Whether to reset the pose of the robot at the start of the
     *                  command
     * @return the path following command
     */
    public Command createPathFollowingCommand(String path, Map<String, Command> events, boolean resetPose) {
        var trajectory = PathPlanner.loadPath(path, SwerveConstants.PATH_CONSTRAINTS);
        return createPathFollowingCommand(trajectory, events, resetPose);
    }

    /**
     * Creates a path following command
     * 
     * @param path The path to follow
     * @return the path following command
     */
    public Command createPathFollowingCommand(String path) {
        return createPathFollowingCommand(path, new HashMap<>());
    }

    /**
     * Creates a path following command
     * 
     * @param points The points to follow (including the current position)
     * @return the path following command
     */
    public Command createPathFollowingCommand(PathPoint... points) {
        if (points.length < 2)
            return null;
        var trajectory = PathPlanner.generatePath(SwerveConstants.PATH_CONSTRAINTS, Arrays.asList(points));
        return createPathFollowingCommand(trajectory, new HashMap<>(), false);
    }

    /**
     * Adds a vision input to the estimated pose of the robot
     * 
     * @param estimatedPose     The estimated pose of the robot by vision
     * @param timeOfMeasurement The time of the vision measurement by
     *                          {@link Timer#getFPGATimestamp()}
     */
    public void addVisionInput(Pose2d estimatedPose, double timeOfMeasurement) {
        poseEstimator.addVisionMeasurement(estimatedPose, timeOfMeasurement);
    }

    /**
     * Gets the roll of the robot
     * 
     * @return The roll of the robot
     */
    public double getRoll() {
        return gyro.getRoll() - startRoll;
    }

    /**
     * Gets the pitch of the robot
     * 
     * @return The pitch of the robot
     */
    public double getPitch() {
        return gyro.getPitch() - startPitch;
    }

    /**
     * Gets the upwards rotation of the robot
     * 
     * @return The upwards rotation of the robot
     */
    public double getUpRotation() {
        double pitch = getPitch();
        double roll = getRoll();
        double sign;
        if (Math.abs(pitch) > Math.abs(roll))
            sign = Math.signum(pitch);
        else
            sign = Math.signum(roll);
        return sign * Math.sqrt(pitch * pitch + roll * roll);
    }

    /**
     * Gets the upwards angular velocity of the robot
     * 
     * @return The upwards angular velocity of the robot
     */
    public double getUpAngularVel() {
        double[] arr = new double[3];
        gyro.getRawGyro(arr);
        double sign;
        if (Math.abs(arr[0]) > Math.abs(arr[1]))
            sign = Math.signum(arr[0]);
        else
            sign = Math.signum(arr[1]);
        return sign * Math.sqrt(arr[0] * arr[0] + arr[1] * arr[1]);
    }

    @Override
    public void periodic() {
        poseEstimator.update(getGyroRotation(), getModulePositions());
        field.setRobotPose(getPose());
        Pair<Pose2d, Double> visionInput = VisionUtils.getVisionPose();
        if (visionInput != null)
            addVisionInput(visionInput.getFirst(), visionInput.getSecond());
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        SmartDashboard.putData("Front Left Module", modules[0]);
        SmartDashboard.putData("Front Right Module", modules[1]);
        SmartDashboard.putData("Back Left Module", modules[2]);
        SmartDashboard.putData("Back Right Module", modules[3]);

        SmartDashboard.putData("Field", field);

        builder.addDoubleProperty("Angle", this::getAngle, null);

        Utils.addDoubleProperty(builder, "UpAngle", this::getUpRotation, 2);
        Utils.addDoubleProperty(builder, "UpAngularVel", this::getUpAngularVel, 2);

        Utils.putData("Change Neutral", "Change", new InstantCommand(this::swapNeutralMode).ignoringDisable(true));

        Utils.putData("Zero Angle", "Zero", new InstantCommand(this::resetAngle).ignoringDisable(true));

        Utils.putData("Calibrate Offsets", "Calibrate", new InstantCommand(() -> {
            for (var module : modules) {
                module.calibrateOffset();
            }
        }).ignoringDisable(true));
    }
}
