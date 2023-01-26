package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Chassis;

public class GoUpRamp extends CommandBase {
    
    private final Chassis chassis;
    private final double startVelocity;
    private double velocity;
    private double angleSign;
    private State lastState;
    private boolean onRamp;
    private int count;

    public GoUpRamp(Chassis chassis, double velocity) {
        this.chassis = chassis;
        this.startVelocity = velocity;

        addRequirements(chassis);
    }

    private enum State {
        POSITIVE,
        NEGATIVE,
        UNKNOWN;

        public boolean differentSign(double sign) {
            sign = Math.signum(deadbandAngle(sign));
            return (sign == 1 && this == NEGATIVE) ||
                    (sign == -1 && this == POSITIVE);
        }
    }

    @Override
    public void initialize() {
        onRamp = false;
        velocity = startVelocity;
        lastState = State.UNKNOWN;
    }

    private static double deadbandAngle(double angle) {
        if (Math.abs(angle) < 1) {
            return 0;
        }
        return angle;
    }

    @Override
    public void execute() {
        double angle = chassis.getUpRotation();
        chassis.setAngleAndVelocity(onRamp && angleSign == 0 ? 0 : velocity, 0, 0);
        
        if (!onRamp && Math.abs(angle) > 5)
            onRamp = true;
        else if (onRamp && lastState.differentSign(angle))
            velocity /= -2;

        angleSign = Math.signum(deadbandAngle(angle));
        if (angleSign == 1)
            lastState = State.POSITIVE;
        else if (angleSign == -1)
            lastState = State.NEGATIVE;

        if (angleSign == 0 && onRamp) {
            count++;
        } else {
            count = 0;
        }
    }

    @Override
    public boolean isFinished() {
        return count >= 50;
    }

    @Override
    public void end(boolean interrupted) {
        chassis.stop();
    }
}
