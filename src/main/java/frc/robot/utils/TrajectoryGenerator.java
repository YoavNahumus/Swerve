package frc.robot.utils;

import java.util.ArrayList;
import java.util.List;

import com.pathplanner.lib.PathPoint;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.Constants;

/**
 * Utility class for generating trajectories
 */
public class TrajectoryGenerator {
    private final List<Pose2d> positions;
    private final List<Rotation2d> headings;
    private final List<Double> velocities;
    private final Alliance alliance;

    /**
     * Constructs a new TrajectoryGenerator
     * 
     * @param alliance The trajectory's alliance
     */
    public TrajectoryGenerator(Alliance alliance) {
        positions = new ArrayList<>();
        headings = new ArrayList<>();
        velocities = new ArrayList<>();
        this.alliance = alliance;
    }

    /**
     * Adds a point to the trajectory
     * 
     * @param robotPosition The robot's position
     * @param heading       The robot's heading
     * @param velocity      The robot's velocity
     */
    public void add(Pose2d robotPosition, Rotation2d heading, double velocity) {
        positions.add(robotPosition);
        headings.add(heading);
        velocities.add(velocity);
    }

    /**
     * Adds a point to the trajectory
     * 
     * @param robotPosition The robot's position
     * @param heading       The robot's heading
     */
    public void add(Pose2d robotPosition, Rotation2d heading) {
        add(robotPosition, heading, -1);
    }

    /**
     * Adds a point to the trajectory
     * 
     * @param robotPosition The robot's position
     */
    public void add(Pose2d robotPosition) {
        add(robotPosition, null);
    }

    /**
     * Calculates the heading of a point based on the previous and next points
     * 
     * @param index The index of the point to calculate the heading of
     * @return The calculated heading
     */
    private Rotation2d calculateHeading(int index) {
        if (index < 0 && index >= positions.size())
            throw new IllegalArgumentException(
                    "Index must be between 0 and " + (positions.size() - 1) + " (inclusive)");
        return positions.get(Math.min(index + 1, positions.size() - 1)).getTranslation()
                .minus(positions.get(Math.max(index - 1, 0)).getTranslation()).getAngle();
    }

    /**
     * Generates the trajectory, converts the points to the current alliance
     * 
     * @param startPosition The robot's starting position to enter the trajectory
     *                      (relative to the field)
     * @return The generated trajectory
     */
    public PathPoint[] generate(Pose2d startPosition) {
        if (alliance != Utils.getAlliance())
            startPosition = new Pose2d(
                    new Translation2d(Constants.FIELD_WIDTH - startPosition.getX(), startPosition.getY()),
                    startPosition.getRotation().rotateBy(Rotation2d.fromDegrees(180)));
        headings.add(0, null);
        positions.add(0, startPosition);
        velocities.add(0, -1.);

        return generate();
    }

    /**
     * Generates the trajectory, converts the points to the current alliance
     * 
     * @return The generated trajectory
     */
    public PathPoint[] generate() {
        PathPoint[] path = new PathPoint[headings.size()];
        for (int i = 0; i < path.length; i++) {
            Pose2d position = positions.get(i);
            Rotation2d heading = headings.get(i);
            double velocity = velocities.get(i);
            if (heading == null)
                heading = calculateHeading(i);
            path[i] = Utils.createAllianceRelativePathPoint(position.getTranslation(), heading,
                    position.getRotation(), velocity, alliance);
        }

        return path;
    }
}
