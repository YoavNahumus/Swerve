package frc.robot.commands;

import com.pathplanner.lib.PathPoint;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.Chassis;
import frc.robot.utils.Utils;

/**
 * This command is used to go to the nodes on the field from the community.
 */
public class GotoNodes extends CommandBase {

    private static final Translation2d[][] NODES = {
            { new Translation2d(1.38, 0.51), new Translation2d(1.38, 1.07), new Translation2d(1.38, 1.63) },
            { new Translation2d(1.38, 2.19), new Translation2d(1.38, 2.75), new Translation2d(1.38, 3.31) },
            { new Translation2d(1.38, 3.87), new Translation2d(1.38, 4.43), new Translation2d(1.38, 4.99) }
    }; // All relative to blue alliance

    // TODO: Fix these values
    /** Distance the robot should be from the node of the cube */
    private static final double DISTANCE_CUBE = 1;
    /** Distance the robot should be from the node of the cone */
    private static final double DISTANCE_CONE = 1;

    /**
     * The position of the robot on the grid.
     */
    public static enum Position {
        BOTTOM, MIDDLE, TOP;

        public int getValue() {
            switch (this) {
                case TOP:
                    return 2;
                case MIDDLE:
                    return 1;
                case BOTTOM:
                    return 0;
                default:
                    return -1;
            }
        }
    }

    private final Chassis chassis;
    private final XboxController controller;
    private Command command;

    private final SendableChooser<Position> gridPositionChooser;
    private Position gridPosition;

    private final SendableChooser<Position> nodePositionChooser;
    private Position nodePosition;

    /**
     * Constructor for the GotoNodes command.
     * 
     * @param chassis
     * @param controller
     */
    public GotoNodes(Chassis chassis, XboxController controller) {
        this.chassis = chassis;
        this.controller = controller;
        gridPositionChooser = new SendableChooser<>();
        nodePositionChooser = new SendableChooser<>();

        initChoosers();
    }

    /**
     * Initialize the sendable choosers.
     */
    private void initChoosers() {
        gridPositionChooser.setDefaultOption("Bottom", Position.BOTTOM);
        gridPositionChooser.addOption("Middle", Position.MIDDLE);
        gridPositionChooser.addOption("Top", Position.TOP);

        nodePositionChooser.setDefaultOption("ConeBottom", Position.BOTTOM);
        nodePositionChooser.addOption("Cube", Position.MIDDLE);
        nodePositionChooser.addOption("ConeTop", Position.TOP);

        SmartDashboard.putData("Grid", gridPositionChooser);
        SmartDashboard.putData("Node", nodePositionChooser);

        Utils.putData("Choose Node", "Choose", new InstantCommand(this::changeTarget).ignoringDisable(true));
    }

    /**
     * Initialize the command.
     */
    private void initCommand() {
        Translation2d node = NODES[gridPosition.getValue()][nodePosition.getValue()];
        if (nodePosition == Position.MIDDLE) {
            node = node.plus(new Translation2d(DISTANCE_CUBE, 0));
        } else {
            node = node.plus(new Translation2d(DISTANCE_CONE, 0));
        }

        command = chassis.createPathFollowingCommand(
                new PathPoint(chassis.getPose().getTranslation(), chassis.getVelocity().getAngle(),
                        chassis.getRotation(), chassis.getVelocity().getNorm()),
                Utils.createAllianceRelativePathPoint(node, Rotation2d.fromDegrees(180), Rotation2d.fromDegrees(180),
                        -1, Alliance.Blue));
    }

    @Override
    public void initialize() {
        initCommand();
        command.schedule();
    }

    /**
     * Changes the target of the command to the target selected in the Smart
     * Dashboard.
     */
    private void changeTarget() {
        gridPosition = gridPositionChooser.getSelected();
        nodePosition = nodePositionChooser.getSelected();

        if (command.isScheduled()) {
            command.cancel();
            initCommand();
            command.schedule();
        }
    }

    @Override
    public void end(boolean interrupted) {
        command.cancel();
        chassis.stop();
    }

    @Override
    public boolean isFinished() {
        return Utils.hasInput(controller) || command.isScheduled();
    }
}
