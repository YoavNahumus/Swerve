package frc.robot.utils;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.XboxController;

public final class General {
    /**
     * Gets the difference between two angles, accounting for wrapping around 360
     * degrees
     * 
     * @param current The current angle, in degrees
     * @param target  The target angle, in degrees
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
     * 
     * @param angle The angle to normalize
     * @return The normalized angle
     */
    public static double normalizeAngle(double angle) {
        return ((angle % 360) + 360) % 360;
    }

    /**
     * Limits a value to be at least greater than 0.1 or less than -0.1
     * 
     * @param value The value to limit
     * @return The limited value
     */
    public static double deadband(double value) {
        if (Math.abs(value) < 0.15) {
            return 0;
        }
        return value;
    }

    /**
     * Scale a value by a power
     * 
     * @param value The value to scale
     * @param scale The power to scale by
     * @return The scaled value
     */
    public static double scale(double value, double scale) {
        return Math.pow(Math.abs(value), scale) * Math.signum(value);
    }

    /**
     * Rounds a double to a certain number of decimal places
     * 
     * @param value  The value to round
     * @param places The number of decimal places to round to
     * @return The rounded value, in a String
     */
    public static String round(double value, int places) {
        long factor = (long) Math.pow(10, places);
        long temp = Math.round(value * factor);
        return String.format("%d.%d", temp / factor, temp % factor);
    }

    /**
     * Adds a double property to a SendableBuilder
     * 
     * @param builder            The SendableBuilder to add the property to
     * @param name               The name of the property
     * @param supplier           The supplier of the property
     * @param placesAfterDecimal The number of decimal places to round to
     */
    public static void addDoubleProperty(SendableBuilder builder, String name, DoubleSupplier supplier,
            int placesAfterDecimal) {
        builder.addStringProperty(name, () -> round(supplier.getAsDouble(), placesAfterDecimal), null);
    }

    /**
     * Gets the scaled x and y values from an XboxController
     * 
     * @param controller The XboxController to get the values from
     * @param scale      The scale to scale by
     * @return The scaled x and y values
     */
    public static Translation2d getScaledXY(XboxController controller, double scale) {
        Translation2d translation = new Translation2d(controller.getLeftX(), controller.getLeftY());
        translation = translation.times(Math.pow(translation.getNorm(), scale - 1));
        return translation;
    }
}
