package frc.robot.commands.templates;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.Subsystem;

/**
 * An {@link InstantCommand} that runs when disabled.
 */
public class InstantCommandInDisable extends InstantCommand {

    /**
     * Creates a new InstantCommandInDisable.
     * 
     * @param initialize   The action to run when the command is initialized
     * @param requirements The subsystems to require
     */
    public InstantCommandInDisable(Runnable initialize, Subsystem... requirements) {
        super(initialize, requirements);
    }

    @Override
    public boolean runsWhenDisabled() {
        return true;
    }
}
