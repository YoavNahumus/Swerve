package frc.robot.utils;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants;
import frc.robot.Constants.VisionConstants;

/**
 * Utility class for vision
 */
public class VisionUtils {

    /**
     * Gets the pose of the robot from the vision system
     * 
     * @return The pose of the robot from the vision system, and the timestamp of
     *         the measurement, or null if no target is found
     */
    public static Pair<Pose2d, Double> getVisionPose() {
        double hasTarget = VisionConstants.LIMELIGHT_TABLE.getEntry("tv").getDouble(0);
        if (hasTarget == 0)
            return null;

        double[] limeLightPose = VisionConstants.LIMELIGHT_TABLE.getEntry("botpose").getDoubleArray(new double[0]);
        if (limeLightPose.length != 6)
            return null;

        double latency = VisionConstants.LIMELIGHT_TABLE.getEntry("tl").getDouble(0);
        limeLightPose[0] = Constants.FIELD_WIDTH / 2 - limeLightPose[0];
        limeLightPose[1] = limeLightPose[1] + Constants.FIELD_HEIGHT / 2;

        Rotation2d robotRotation = Rotation2d.fromDegrees(limeLightPose[5]);
        Translation2d robotTranslation = new Translation2d(limeLightPose[0], limeLightPose[1])
                .minus(VisionConstants.CAMERA_OFFSET.rotateBy(robotRotation));

        return new Pair<Pose2d, Double>(
                new Pose2d(robotTranslation, robotRotation),
                Timer.getFPGATimestamp() - ((latency + VisionConstants.CAPTURE_LATENCY) / 1000));
    }
}
