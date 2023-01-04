package frc.robot.commands.templates;

import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.Subsystem;

public class RunCommandInDisable extends RunCommand {
    public RunCommandInDisable(Runnable execute, Subsystem... requirements) {
        super(execute, requirements);
    }

    @Override
    public boolean runsWhenDisabled() {
        return true;
    }
}
