package frc.robot.utils;

public class FeedForward {
    private double kS;
    private double kV;
    private double kA;

    public FeedForward(double kS, double kV, double kA){
        this.kA = kA;
        this.kS = kS;
        this.kV = kV;
    }

    public double calc(double vel, double accel){
        return kS * Math.signum(vel) + kV * vel + accel * kA;
    }

    public double calc(double vel){
        return calc(vel, 0);
    }
}
