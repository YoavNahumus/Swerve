package frc.robot.utils;

public final class General {
    /**
     * Gets the difference between two angles, accounting for wrapping around 360 degrees
     * @param current The current angle, in degrees
     * @param target The target angle, in degrees
     * @return The difference between the two angles, between -180 and 180 degrees
     */
    public static double getAngleDifference(double current, double target) {
        double difference = target - current;
        difference %= 360;
        if (difference > 180) {
            difference -= 360;
        } else if (difference < -180) {
            difference += 360;
        }
        return difference;
    }

    /**
     * Normalizes an angle to be between 0 and 360 degrees
     * @param angle The angle to normalize
     * @return The normalized angle
     */
    public static double normalizeAngle(double angle) {
        return ((angle % 360) + 360) % 360;
    }
}
