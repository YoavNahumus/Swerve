package frc.robot.utils;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.Sendable;
import edu.wpi.first.wpilibj.smartdashboard.SendableBuilder;
import frc.robot.Constants;

public class SwerveUnit implements Sendable{
    private final WPI_TalonFX dirMotor;
    private final WPI_TalonFX moveMotor;

    private boolean isInverted;

    private static final int pulseInRev = Constants.ch_pulseInRev;
    private static final double pulseToMeter = Constants.ch_pulseToMeter;
    private static final double pulsemsToMetersec = pulseToMeter * 10;
    private static final FeedForward aff = new FeedForward(Constants.ch_kS, Constants.ch_kV, 0);


    public SwerveUnit(int dirMotorPort, int moveMotorPort){
        dirMotor = new WPI_TalonFX(dirMotorPort);
        moveMotor = new WPI_TalonFX(moveMotorPort);

        moveMotor.config_kD(0, Constants.ch_velKd);
        moveMotor.config_kI(0, Constants.ch_velKi);
        moveMotor.config_kP(0, Constants.ch_velKp);
        
        dirMotor.config_kP(0, Constants.ch_angleKp);
        dirMotor.config_kI(0, Constants.ch_angleKi);
        dirMotor.config_kD(0, Constants.ch_angleKd);
        isInverted = false;
    }

    /**
     * @return angle in degrees (could be > 360)
     */
    public double getAngle(){
        return getDirEncoder() * 360. / pulseInRev;
    }

    /**
     * @return the raw encoder output for the directional motor
     */
    public double getDirEncoder(){
        return dirMotor.getSelectedSensorPosition();
    }

    /**
     * @return the velocity in m/s
     */
    public double getVelocity(){
        return moveMotor.getSelectedSensorVelocity() * pulsemsToMetersec;
    }

    public void setMoveInverted(boolean isInverted){
        this.isInverted = isInverted;
        moveMotor.setInverted(isInverted);
    }

    public void setDirInverted(boolean isInverted){
        dirMotor.setInverted(isInverted);
    }

    public void setMoveSensorPhase(boolean isInverted){
        moveMotor.setSensorPhase(isInverted);
    }

    public void setDirSensorPhase(boolean isInverted){
        dirMotor.setSensorPhase(isInverted);
    }

    public void setVelocity(double vel){
        moveMotor.set(ControlMode.Velocity, vel / pulsemsToMetersec, DemandType.ArbitraryFeedForward, aff.calc(vel));
    }

    public void setRelVelocity(double vel){
        setVelocity(vel);
    }

    /**
     * sets the angle of the wheel to the desired angle in the fastest way posible.
     * sometimes will make the wheel move backwards
     * @param angle in the range [0,360)
     */
    public void setAngleSmart(double angle){
        double currentAngle = getAngle();
        double relAngle = angle + 360 * (int)(currentAngle / 360) - (currentAngle < 0 ? 360 : 0);
        double diff = currentAngle - relAngle;

        if (diff > 180){
            relAngle += 360;
        }
        else if (diff < -180){
            relAngle -= 360;
        }

        if (diff > 90){
            relAngle += 180;
            moveMotor.setInverted(!isInverted);
        }
        else if (diff < -90){
            relAngle -= 180;
            moveMotor.setInverted(!isInverted);
        }
        else moveMotor.setInverted(isInverted);

        dirMotor.set(ControlMode.Position, relAngle * pulseInRev / 360.);
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.addDoubleProperty("Angle", this::getAngle, null);
        builder.addDoubleProperty("Velocity", this::getVelocity, null);
    }
}
