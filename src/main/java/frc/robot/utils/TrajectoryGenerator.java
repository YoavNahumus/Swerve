package frc.robot.utils;

import java.util.ArrayList;
import java.util.List;

import com.pathplanner.lib.PathPoint;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

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
     * Generates the trajectory, converts the points to the current alliance
     * 
     * @param startPosition The robot's starting position to enter the trajectory
     * @return The generated trajectory
     */
    public PathPoint[] generate(Pose2d startPosition) {
        headings.add(0, positions.get(0).getTranslation().minus(startPosition.getTranslation()).getAngle());
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
            path[i] = Utils.createAllianceRelativePathPoint(positions.get(i).getTranslation(), headings.get(i),
                    positions.get(i).getRotation(), velocities.get(i), i == 0 ? Utils.getAlliance() : alliance);
        }

        return path;
    }
}
