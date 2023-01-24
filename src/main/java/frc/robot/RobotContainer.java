// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.HashMap;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.Drive;
import frc.robot.commands.GotoCommunity;
import frc.robot.commands.GotoLoadingZone;
import frc.robot.commands.GotoNodes;
import frc.robot.subsystems.Chassis;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
    private final XboxController controller = new XboxController(0);
    private final JoystickButton aButton = new JoystickButton(controller, 1);
    private final JoystickButton bButton = new JoystickButton(controller, 2);
    private final Chassis chassis;
    private static RobotContainer instance;

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    private RobotContainer() {
        chassis = new Chassis();
        chassis.setDefaultCommand(new Drive(chassis, controller));

        configureButtonBindings();
    }

    /**
     * Returns the instance of the RobotContainer class.
     * @return the instance of the RobotContainer class.
     */
    public static RobotContainer getInstance() {
        if (instance == null) {
            instance = new RobotContainer();
        }
        return instance;
    }

    /**
     * Use this method to define your button->command mappings. Buttons can be
     * created by
     * instantiating a {@link GenericHID} or one of its subclasses ({@link Joystick}
     * or {@link XboxController}), and then passing it to a {@link JoystickButton}.
     */
    private void configureButtonBindings() {
        aButton.onTrue(new GotoLoadingZone(chassis, controller));
        bButton.onTrue(new GotoCommunity(chassis, controller).andThen(new GotoNodes(chassis, controller)));
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        return chassis.createPathFollowingCommand("Test2", new HashMap<>(), true);
    }
}
