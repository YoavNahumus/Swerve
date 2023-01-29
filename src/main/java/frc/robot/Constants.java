// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.PathConstraints;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.robot.utils.Rectangle;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    public static final double JOYSTICK_DEADBAND = 0.1; // the deadband for the joysticks
    public static final double JOYSTICK_ANGLE_DEADBAND = 0.2; // the deadband for the angle of the joysticks
    public static final double JOYSTICK_IDLE_DEADBAND = 0.3; // the deadband to check if the joystick is idle

    public static final Rectangle RAMP = new Rectangle(2.91, 1.51, 4.85, 3.98); // in meters, blue alliance
    public static final Rectangle OPEN_AREA = new Rectangle(4.85, 0.0, 11.69, 8.02); // in meters, blue alliance
    public static final Rectangle ENTRANCE_BOTTOM = new Rectangle(2.91, 0.0, 4.85, 1.51); // in meters, blue alliance
    public static final Rectangle ENTRANCE_TOP = new Rectangle(2.91, 3.98, 4.85, 5.49); // in meters, blue alliance
    public static final Rectangle COMMUNITY_BOTTOM = new Rectangle(0.0, 0.0, 2.91, 1.51); // in meters, blue alliance
    public static final Rectangle COMMUNITY_TOP = new Rectangle(0.0, 3.98, 2.91, 5.49); // in meters, blue alliance
    public static final Rectangle COMMUNITY_MIDDLE = new Rectangle(0, 1.51, 2.91, 3.98); // in meters, blue alliance
    public static final Rectangle LOADING_ZONE = new Rectangle(11.69, 5.55, 16.54, 8.02); // in meters, blue alliance

    public static final double FIELD_WIDTH = 16.54; // in meters
    public static final double FIELD_HEIGHT = 8.02; // in meters

    /**
     * The Swerve Modules constants.
     */
    public static class SwerveModuleConstants {
        public final double angleOffset;
        public final int moveMotorID;
        public final int angleMotorID;
        public final int absoluteEncoderID;

        public static final double VELOCITY_KP = 0.1;
        public static final double VELOCITY_KS = 0.0479;
        public static final double VELOCITY_KV = 0.22185;
        public static final SimpleMotorFeedforward VELOCITY_FF = new SimpleMotorFeedforward(VELOCITY_KS, VELOCITY_KV);
        public static final double ANGLE_KP = 0.2;
        public static final double ANGLE_KI = 0.0013;

        public static final double PPR_FALCON = 2048;
        public static final double WHEEL_PERIMITER = 0.1016 * Math.PI; // meters
        public static final double GEAR_RATIO_VEL = 8.14;
        public static final double PULSE_PER_METER = PPR_FALCON * GEAR_RATIO_VEL / WHEEL_PERIMITER;

        public static final double GEAR_RATIO_ANGLE = 12.8;
        public static final double PULSE_PER_DEGREE = PPR_FALCON * GEAR_RATIO_ANGLE / 360;

        /**
         * Creates a new SwerveModuleConstants.
         * 
         * @param angleOffset       The offset of the absolute encoder from the module
         * @param moveMotorID       The CAN ID of the drive motor
         * @param angleMotorID      The CAN ID of the angle motor
         * @param absoluteEncoderID The CAN ID of the absolute encoder
         */
        private SwerveModuleConstants(double angleOffset, int moveMotorID, int angleMotorID, int absoluteEncoderID) {
            this.angleOffset = angleOffset;
            this.moveMotorID = moveMotorID;
            this.angleMotorID = angleMotorID;
            this.absoluteEncoderID = absoluteEncoderID;
        }

        public static final SwerveModuleConstants FRONT_LEFT = new SwerveModuleConstants(0.703125, 7, 8, 11);
        public static final SwerveModuleConstants FRONT_RIGHT = new SwerveModuleConstants(303.134765625, 5, 6, 13);
        public static final SwerveModuleConstants BACK_LEFT = new SwerveModuleConstants(224.736328125, 1, 2, 10);
        public static final SwerveModuleConstants BACK_RIGHT = new SwerveModuleConstants(109.423828125, 3, 4, 12);
    }

    /**
     * The Swerve Drive constants.
     */
    public static final class SwerveConstants {
        public static final SwerveDriveKinematics KINEMATICS = new SwerveDriveKinematics(
                new Translation2d(0.26515, 0.2215), // front left
                new Translation2d(0.26515, -0.2215), // front right
                new Translation2d(-0.26515, 0.2215), // back left
                new Translation2d(-0.26515, -0.2215) // back right
        );

        public static final int GYRO_ID = 14;

        public static final double MAX_SPEED = (1 - SwerveModuleConstants.VELOCITY_KS)
                / SwerveModuleConstants.VELOCITY_KV; // meters per second
        public static final double MAX_ACCELERATION = 3; // meters per second squared
        public static final PathConstraints PATH_CONSTRAINTS = new PathConstraints(MAX_SPEED, MAX_ACCELERATION);
        public static final double MAX_DRIVE_SPEED = 3.5;
        public static final double MAX_ANGULAR_SPEED = 2 * Math.PI; // radians per second

        public static final double AUTO_TRANSLATION_KP = 1;
        public static final double AUTO_TRANSLATION_KI = 0;
        public static final double AUTO_ROTATION_KP = 1;
        public static final double AUTO_ROTATION_KI = 0;

        public static final double TELEOP_ROTATION_KP = 4;
        public static final double TELEOP_ROTATION_KI = 0.3;

        public static final double ANGLE_TOLERANCE = Math.PI / 120;
    }

    /**
     * The Vision constants.
     */
    public static final class VisionConstants {
        public static final NetworkTable LIMELIGHT_TABLE = NetworkTableInstance.getDefault().getTable("limelight");
        public static final double CAPTURE_LATENCY = 11; // ms
        public static final Translation2d CAMERA_OFFSET = new Translation2d(0.3, 0.0); // in meters
    }
}
