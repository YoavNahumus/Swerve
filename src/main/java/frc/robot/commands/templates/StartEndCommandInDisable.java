package frc.robot.commands.templates;

import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.Subsystem;

public class StartEndCommandInDisable extends StartEndCommand {
    public StartEndCommandInDisable(Runnable initialize, Runnable onEnd, Subsystem... requirements) {
        super(initialize, onEnd, requirements);
    }

    @Override
    public boolean runsWhenDisabled() {
        return true;
    }
}
