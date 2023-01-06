// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;

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
    public static class SwerveModuleConstants {
        public final double kS, kV, angleOffset;
        public final int moveMotorID, angleMotorID, absoluteEncoderID;

        public static final double VELOCITY_KP = 0.1;
        public static final double ANGLE_KP = 0.2;
        public static final double ANGLE_KI = 0.0013;

        public static final double PPR_FALCON = 2048;
        public static final double WHEEL_PERIMITER = 0.1016 * Math.PI; // meters
        public static final double GEAR_RATIO_VEL = 8.14;
        public static final double PULSE_PER_METER = PPR_FALCON * GEAR_RATIO_VEL / WHEEL_PERIMITER;

        public static final double GEAR_RATIO_ANGLE = 12.8;
        public static final double PULSE_PER_DEGREE = PPR_FALCON * GEAR_RATIO_ANGLE / 360;

        private SwerveModuleConstants(double angleOffset, int moveMotorID, int angleMotorID, int absoluteEncoderID, double kS, double kV) {
            this.angleOffset = angleOffset;
            this.moveMotorID = moveMotorID;
            this.angleMotorID = angleMotorID;
            this.absoluteEncoderID = absoluteEncoderID;
            this.kS = kS;
            this.kV = kV;
        }

        public static final SwerveModuleConstants FRONT_LEFT = new SwerveModuleConstants(303.31, 7, 8, 11, 0.0479, 0.22185);
        public static final SwerveModuleConstants FRONT_RIGHT = new SwerveModuleConstants(312.18, 5, 6, 13, 0.0479, 0.22185);
        public static final SwerveModuleConstants BACK_LEFT = new SwerveModuleConstants(228.69, 1, 2, 10, 0.0479, 0.22185);
        public static final SwerveModuleConstants BACK_RIGHT = new SwerveModuleConstants(108.8, 3, 4, 12, 0.0479, 0.22185);
    }

    public static final class SwerveConstants {
        public static final SwerveDriveKinematics KINEMATICS = new SwerveDriveKinematics(
                new Translation2d(0.31515, 0.27015), // front left
                new Translation2d(0.31515, -0.27015), // front right
                new Translation2d(-0.31515, 0.27015), // back left
                new Translation2d(-0.31515, -0.27015) // back right
        );

        public static final int GYRO_ID = 14;
        
        public static final double MAX_SPEED = 4.25; // meters per second
        public static final double MAX_DRIVE_SPEED = 3.5;
        public static final double MAX_ANGULAR_SPEED = 2 * Math.PI; // radians per second
    }
}
