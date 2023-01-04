package frc.robot.commands.templates;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.Subsystem;

public class InstantCommandInDisable extends InstantCommand {
    public InstantCommandInDisable(Runnable initialize, Subsystem... requirements) {
        super(initialize, requirements);
    }

    @Override
    public boolean runsWhenDisabled() {
        return true;
    }
}
