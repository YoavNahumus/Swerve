package frc.robot.commands.templates;

import java.util.function.BooleanSupplier;
import java.util.function.Consumer;

import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.Subsystem;

public class FunctionalCommandInDisable extends FunctionalCommand {
    public FunctionalCommandInDisable(Runnable initialize, Runnable execute, Consumer<Boolean> onEnd,
            BooleanSupplier isFinished, Subsystem... requirements) {
        super(initialize, execute, onEnd, isFinished, requirements);
    }

    @Override
    public boolean runsWhenDisabled() {
        return true;
    }
}
