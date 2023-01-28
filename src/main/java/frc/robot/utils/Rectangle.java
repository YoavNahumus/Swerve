package frc.robot.utils;

import edu.wpi.first.math.geometry.Translation2d;

/**
 * Represents a rectangle in a 2D plane
 */
public class Rectangle {
    private final Translation2d bottomLeft, topRight;

    /**
     * Creates a new rectangle with the given bottom left and top right corners
     * 
     * @param bottomLeft The bottom left corner
     * @param topRight   The top right corner
     */
    public Rectangle(Translation2d bottomLeft, Translation2d topRight) {
        this.bottomLeft = bottomLeft;
        this.topRight = topRight;
    }

    /**
     * Creates a new rectangle with the given bottom left and top right corners
     * 
     * @param x1 The x coordinate of the bottom left corner
     * @param y1 The y coordinate of the bottom left corner
     * @param x2 The x coordinate of the top right corner
     * @param y2 The y coordinate of the top right corner
     */
    public Rectangle(double x1, double y1, double x2, double y2) {
        this(new Translation2d(x1, y1), new Translation2d(x2, y2));
    }

    /**
     * Checks if a point is inside the rectangle
     * 
     * @param point The point to check
     * @return Whether the point is inside the rectangle
     */
    public boolean isInside(Translation2d point) {
        return point.getX() >= bottomLeft.getX() && point.getX() <= topRight.getX()
                && point.getY() >= bottomLeft.getY() && point.getY() <= topRight.getY();
    }

    /**
     * Gets the center of the rectangle
     * 
     * @return The center of the rectangle
     */
    public Translation2d getCenter() {
        return new Translation2d((topRight.getX() + bottomLeft.getX()) / 2, (topRight.getY() + bottomLeft.getY()) / 2);
    }
}
