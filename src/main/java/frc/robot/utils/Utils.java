package frc.robot.utils;

import java.util.function.DoubleSupplier;

import com.pathplanner.lib.PathPoint;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;

/**
 * Contains general utility methods
 */
public final class Utils {
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
    public static double normalizeDegrees(double angle) {
        return ((angle % 360) + 360) % 360;
    }

    /**
     * Normalizes an angle to be between 0 and 2pi radians
     * 
     * @param angle The angle to normalize
     * @return The normalized angle
     */
    public static double normalizeRadians(double angle) {
        return Math.toRadians(normalizeDegrees(Math.toDegrees(angle)));
    }

    /**
     * Limits a value to be at least greater than the deadband constant or smaller
     * than -deadband constant
     * 
     * @param value The value to limit
     * @return The limited value
     */
    public static double deadband(double value) {
        if (Math.abs(value) < Constants.JOYSTICK_DEADBAND) {
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
        return String.format("%." + places + "f", value);
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
     * The side of the controller, Left or Right
     */
    public static enum ControllerSide {
        LEFT, RIGHT
    }

    /**
     * Gets the x and y values from an XboxController
     * 
     * @param controller The XboxController to get the values from
     * @param side       The side of the controller to get the values from
     * @param deadband   Whether to apply a deadband to the values
     * @return The x and y values (positive is right and up)
     */
    public static Translation2d getStick(XboxController controller, ControllerSide side, boolean deadband) {
        if (side == ControllerSide.LEFT) {
            return deadband ? new Translation2d(deadband(controller.getLeftX()), deadband(-controller.getLeftY()))
                    : new Translation2d(controller.getLeftX(), -controller.getLeftY());
        }
        return deadband ? new Translation2d(deadband(controller.getRightX()), deadband(-controller.getRightY()))
                : new Translation2d(controller.getRightX(), -controller.getRightY());
    }

    /**
     * Gets the scaled x and y values from an XboxController
     * 
     * @param controller The XboxController to get the values from
     * @param side       The side of the controller to get the values from
     * @param scale      The scale to scale by
     * @return The scaled x and y values (positive is right and up)
     */
    public static Translation2d getScaledStick(XboxController controller, ControllerSide side, double scale) {
        Translation2d translation = getStick(controller, side, true);
        translation = translation.times(Math.pow(translation.getNorm(), scale - 1));
        return translation;
    }

    /**
     * Gets the rotation of the right stick on the controller
     * 
     * @param controller The controller to get the value from
     * @param side       The side of the controller to get the value from
     * @return The rotation, 0 is right, positive is counterclockwise (null if the
     *         stick is not being used)
     */
    public static Rotation2d getStickRotation(XboxController controller, ControllerSide side) {
        Translation2d translation = getStick(controller, side, true);
        if (translation.getNorm() <= Constants.JOYSTICK_ANGLE_DEADBAND) {
            return null;
        }
        return translation.getAngle();
    }

    /**
     * Gets the scaled Difference between the two triggers on the controller
     * 
     * @param controller The controller to get the value from
     * @param positive   The side of the controller which is positive output, the
     *                   other being negative
     * @param scale      The scale to scale by
     * @return The scaled difference between the two triggers
     */
    public static double getScaledTriggerDiff(XboxController controller, ControllerSide positive, double scale) {
        if (positive == ControllerSide.LEFT) {
            return scale(deadband(controller.getLeftTriggerAxis()) - deadband(controller.getRightTriggerAxis()), scale);
        }
        return scale(deadband(controller.getRightTriggerAxis()) - deadband(controller.getLeftTriggerAxis()), scale);
    }

    /**
     * Checks if the robot's alliance is the red alliance from the FMSInfo (note: if
     * not in a match, your alliance will be what you have in the DS)
     * 
     * @return true if red, false if blue
     */
    public static boolean isRedAlliance() {
        return getAlliance() == Alliance.Red;
    }

    /**
     * The zone the robot is in
     */
    public static enum Zone {
        LOADING_ZONE, OPEN_AREA, COMMUNITY_TOP, COMMUNITY_MIDDLE, COMMUNITY_BOTTOM, COMMUNITY_ENTRANCE_TOP,
        COMMUNITY_ENTRANCE_BOTTOM;

        /**
         * Gets the zone the robot is in from its position
         * 
         * @param robotPosition The robot's position
         * @return The zone the robot is in
         */
        public static Zone fromRobotLocation(Translation2d robotPosition) {
            if (isRedAlliance())
                robotPosition = new Translation2d(Constants.FIELD_WIDTH - robotPosition.getX(), robotPosition.getY());
            if (Constants.COMMUNITY_BOTTOM.isInside(robotPosition))
                return COMMUNITY_BOTTOM;
            if (Constants.COMMUNITY_MIDDLE.isInside(robotPosition))
                return COMMUNITY_MIDDLE;
            if (Constants.COMMUNITY_TOP.isInside(robotPosition))
                return COMMUNITY_TOP;
            if (Constants.ENTRANCE_BOTTOM.isInside(robotPosition))
                return COMMUNITY_ENTRANCE_BOTTOM;
            if (Constants.ENTRANCE_TOP.isInside(robotPosition))
                return COMMUNITY_ENTRANCE_TOP;
            if (Constants.LOADING_ZONE.isInside(robotPosition))
                return LOADING_ZONE;
            return OPEN_AREA;
        }
    }

    /**
     * The alliance the robot is on
     * 
     * @return The alliance the robot is on
     */
    public static Alliance getAlliance() {
        return DriverStation.getAlliance();
    }

    /**
     * Creates a path point with the position and heading relative to the alliance
     * 
     * @param position          The position of the point
     * @param heading           The heading of the point
     * @param holonomicRotation The holonomic rotation of the point
     * @param velocity          The velocity of the point, -1 for default
     * @param alliance          The alliance the point is relative to
     * @return The path point, with the position and heading relative to the
     *         alliance
     */
    public static PathPoint createAllianceRelativePathPoint(Translation2d position, Rotation2d heading,
            Rotation2d holonomicRotation, double velocity, Alliance alliance) {

        if (getAlliance() != alliance) {
            position = new Translation2d(Constants.FIELD_WIDTH - position.getX(), position.getY());
            heading = heading.rotateBy(Rotation2d.fromDegrees(180));
            holonomicRotation = holonomicRotation.rotateBy(Rotation2d.fromDegrees(180));
            if (velocity > 0)
                velocity = -velocity;
        }
        return new PathPoint(position, heading, holonomicRotation, velocity);
    }

    /**
     * Checks for an input in the controller's joysticks and triggers
     * 
     * @param controller The controller to check
     * @return true if there is an input, false if not
     */
    public static boolean hasInput(XboxController controller) {
        return Math.abs(controller.getLeftX()) > Constants.JOYSTICK_IDLE_DEADBAND
                || Math.abs(controller.getLeftY()) > Constants.JOYSTICK_IDLE_DEADBAND
                || Math.abs(controller.getRightX()) > Constants.JOYSTICK_IDLE_DEADBAND
                || Math.abs(controller.getRightY()) > Constants.JOYSTICK_IDLE_DEADBAND
                || Math.abs(controller.getLeftTriggerAxis()) > Constants.JOYSTICK_IDLE_DEADBAND
                || Math.abs(controller.getRightTriggerAxis()) > Constants.JOYSTICK_IDLE_DEADBAND;
    }

    /**
     * Puts a Sendable to the SmartDashboard with the given key and name
     * 
     * @param key   The key to put the Sendable to
     * @param name  The name of the Sendable
     * @param value The Sendable to put
     */
    public static void putData(String key, String name, Sendable value) {
        SendableRegistry.add(value, name);
        SmartDashboard.putData(key, value);
    }
}
