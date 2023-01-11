package frc.robot.commands.templates;

import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.Subsystem;

/**
 * A {@link StartEndCommand} that runs when disabled.
 */
public class StartEndCommandInDisable extends StartEndCommand {

    /**
     * Creates a new StartEndCommandInDisable.
     * 
     * @param initialize   The action to run when the command is initialized
     * @param onEnd        The action to run when the command ends
     * @param requirements The subsystems to require
     */
    public StartEndCommandInDisable(Runnable initialize, Runnable onEnd, Subsystem... requirements) {
        super(initialize, onEnd, requirements);
    }

    @Override
    public boolean runsWhenDisabled() {
        return true;
    }
}
