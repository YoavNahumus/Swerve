package frc.robot.commands.templates;

import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.Subsystem;

/**
 * A {@link RunCommand} that runs when disabled.
 */
public class RunCommandInDisable extends RunCommand {

    /**
     * Creates a new RunCommandInDisable.
     * 
     * @param execute      The action to run
     * @param requirements The subsystems to require
     */
    public RunCommandInDisable(Runnable execute, Subsystem... requirements) {
        super(execute, requirements);
    }

    @Override
    public boolean runsWhenDisabled() {
        return true;
    }
}
