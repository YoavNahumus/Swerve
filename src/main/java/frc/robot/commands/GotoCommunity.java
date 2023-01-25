package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.Chassis;
import frc.robot.utils.TrajectoryGenerator;
import frc.robot.utils.Utils;
import frc.robot.utils.Utils.Zone;

/**
 * Drives the robot semi autonomously to the community zone.
 */
public class GotoCommunity extends CommandBase {

    private final Chassis chassis;
    private final XboxController controller;
    private Command command;

    /**
     * Constructs a new GotoCommunity command.
     * 
     * @param chassis    The chassis subsystem
     * @param controller The controller to use for input
     */
    public GotoCommunity(Chassis chassis, XboxController controller) {
        this.chassis = chassis;
        this.controller = controller;
    }

    @Override
    public void initialize() {
        command = new InstantCommand();
        TrajectoryGenerator generator = new TrajectoryGenerator(Alliance.Blue);

        Zone zone = Zone.fromRobotLocation(chassis.getPose().getTranslation());
        if (zone == Zone.COMMUNITY_BOTTOM || zone == Zone.COMMUNITY_TOP || zone == Zone.COMMUNITY_MIDDLE) {
            return;
        }

        if (chassis.getPose().getY() > 1.51) {
            // Enter through the top
            switch (zone) {
                case LOADING_ZONE:
                    generator.add(new Pose2d(new Translation2d(10.98, 7.34), Rotation2d.fromDegrees(180)),
                            Rotation2d.fromDegrees(180));
                case OPEN_AREA:
                    generator.add(new Pose2d(new Translation2d(5.65, 4.735), Rotation2d.fromDegrees(180)),
                            Rotation2d.fromDegrees(180));
                default:
                    generator.add(new Pose2d(new Translation2d(2.17, 4.735), Rotation2d.fromDegrees(180)),
                            Rotation2d.fromDegrees(180));
            }
        } else {
            // Enter through the bottom
            generator.add(new Pose2d(new Translation2d(2.17, 0.755), Rotation2d.fromDegrees(180)),
                    Rotation2d.fromDegrees(180));
        }

        command = chassis.createPathFollowingCommand(generator.generate(chassis.getPose()));

        command.schedule();
    }

    @Override
    public boolean isFinished() {
        return !command.isScheduled() || Utils.hasInput(controller);
    }

    @Override
    public void end(boolean interrupted) {
        command.cancel();
        chassis.stop();
    }
}
