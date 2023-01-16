package frc.robot.commands;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashMap;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPoint;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Constants.SwerveConstants;
import frc.robot.subsystems.Chassis;
import frc.robot.utils.General;
import frc.robot.utils.General.Zone;

public class GotoLoadingZone extends CommandBase {

    private final Chassis chassis;
    private final XboxController controller;
    private Command command;

    private static final PathPlannerTrajectory FROM_BOTTOM_BLUE = PathPlanner.loadPath("BottomToFeederBlue",
            SwerveConstants.PATH_CONSTRAINTS),
            FROM_BOTTOM_RED = PathPlanner.loadPath("BottomToFeederRed", SwerveConstants.PATH_CONSTRAINTS),
            FROM_TOP_BLUE = PathPlanner.loadPath("TopToFeederBlue", SwerveConstants.PATH_CONSTRAINTS),
            FROM_TOP_RED = PathPlanner.loadPath("TopToFeederRed", SwerveConstants.PATH_CONSTRAINTS),
            FROM_MIDDLE_BLUE = PathPlanner.generatePath(SwerveConstants.PATH_CONSTRAINTS, Arrays.asList(
                    new PathPoint(new Translation2d(10.98, 7.34), new Rotation2d()),
                    new PathPoint(new Translation2d(15.46, 7.34), new Rotation2d()))),
            FROM_MIDDLE_RED = PathPlanner.generatePath(SwerveConstants.PATH_CONSTRAINTS, Arrays.asList(
                    new PathPoint(new Translation2d(5.56, 7.34), new Rotation2d()),
                    new PathPoint(new Translation2d(1.08, 7.34), new Rotation2d()))),
            FROM_BOTTOM_ENTRANCE_BLUE = PathPlanner.loadPath("BottomEntranceToFeederBlue",
                    SwerveConstants.PATH_CONSTRAINTS),
            FROM_BOTTOM_ENTRANCE_RED = PathPlanner.loadPath("BottomEntranceToFeederRed",
                    SwerveConstants.PATH_CONSTRAINTS);

    public GotoLoadingZone(Chassis chassis, XboxController controller) {
        this.chassis = chassis;
        this.controller = controller;
    }

    @Override
    public void initialize() {
        ArrayList<PathPoint> path = new ArrayList<>();
        path.add(new PathPoint(chassis.getPose().getTranslation(), chassis.getVelocity().getAngle(),
                chassis.getRotation(), chassis.getVelocity().getNorm()));

        Zone zone = Zone.fromRobotLocation(chassis.getPose().getTranslation());

        PathPlannerTrajectory trajectory;
        Command pathCommand = new InstantCommand();
        boolean red = General.isRedAlliance();
        switch (zone) {
            case COMMUNITY_BOTTOM:
                trajectory = red ? FROM_BOTTOM_RED : FROM_BOTTOM_BLUE;
                break;
            case COMMUNITY_TOP:
                trajectory = red ? FROM_TOP_RED : FROM_TOP_BLUE;
                break;
            case COMMUNITY_ENTRANCE_BOTTOM:
                trajectory = red ? FROM_BOTTOM_ENTRANCE_RED : FROM_BOTTOM_ENTRANCE_BLUE;
                break;
            case COMMUNITY_ENTRANCE_TOP:
            case OPEN_AREA:
                trajectory = red ? FROM_MIDDLE_RED : FROM_MIDDLE_BLUE;
                break;
            case LOADING_ZONE:
            default:
                trajectory = null;
                path.add(General.createAllianceRelativePathPoint(new Translation2d(15.46, 7.34), new Rotation2d(),
                        new Rotation2d(), -1, Alliance.Blue));
                break;
        }

        if (trajectory != null) {
            path.add(new PathPoint(trajectory.getInitialPose().getTranslation(), trajectory.getInitialPose().getRotation()));
            pathCommand = chassis.createPathFollowingCommand(trajectory, new HashMap<>(), false);
        }

        command = chassis.createPathFollowingCommand(PathPlanner.generatePath(SwerveConstants.PATH_CONSTRAINTS, path),
                new HashMap<>(), false).andThen(pathCommand);

        command.schedule();
    }

    @Override
    public boolean isFinished() {
        return !command.isScheduled() || General.hasInput(controller);
    }

    @Override
    public void end(boolean interrupted) {
        command.cancel();
    }
}
