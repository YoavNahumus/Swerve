package frc.robot.commands.templates;

import java.util.function.BooleanSupplier;
import java.util.function.Consumer;

import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.Subsystem;

/**
 * A {@link FunctionalCommand} that runs when disabled.
 */
public class FunctionalCommandInDisable extends FunctionalCommand {

    /**
     * Creates a new FunctionalCommandInDisable.
     * 
     * @param initialize   The action to run when the command is initialized
     * @param execute      The action to run periodically
     * @param onEnd        The action to run when the command ends
     * @param isFinished   Whether the command has finished
     * @param requirements The subsystems to require
     */
    public FunctionalCommandInDisable(Runnable initialize, Runnable execute, Consumer<Boolean> onEnd,
            BooleanSupplier isFinished, Subsystem... requirements) {
        super(initialize, execute, onEnd, isFinished, requirements);
    }

    @Override
    public boolean runsWhenDisabled() {
        return true;
    }
}
