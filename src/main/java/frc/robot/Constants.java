// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    //TODO : check constants

    //Measurements
    public static final int ch_pulseInRev = 4096;
    public static final double ch_pulseToMeter = -1;
    public static final double ch_trackWidth = -1;
    public static final double ch_trackLength = -1;
    public static final double ch_trackDiagonal = Math.sqrt(ch_trackWidth * ch_trackWidth + ch_trackLength * ch_trackLength);

    //Constants
    public static final double ch_kS = -1;
    public static final double ch_kV = -1;
    public static final double ch_velKp = -1;
    public static final double ch_velKi = -1;
    public static final double ch_velKd = -1;
    public static final double ch_angleKp = -1;
    public static final double ch_angleKi = -1;
    public static final double ch_angleKd = -1;

    //Ports
    public static final int ch_moveMotorTL = -1;
    public static final int ch_moveMotorTR = -1;
    public static final int ch_moveMotorBL = -1;
    public static final int ch_moveMotorBR = -1;

    public static final int ch_dirMotorTL = -1;
    public static final int ch_dirMotorTR = -1;
    public static final int ch_dirMotorBL = -1;
    public static final int ch_dirMotorBR = -1;

    public static final int ch_gyroPort = -1;
}
