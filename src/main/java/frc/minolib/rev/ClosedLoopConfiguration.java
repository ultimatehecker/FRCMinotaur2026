package frc.minolib.rev;

import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.MAXMotionConfig;

public class ClosedLoopConfiguration {
    public final double kP;
    public final double kI;
    public final double kD;
    public final double mV;
    public final double mA;

    public ClosedLoopConfiguration() {
        this(0.0, 0.0, 0.0, 0.0, 0.0);
    }

    public ClosedLoopConfiguration(final double kP, final double kI, final double kD) {
        this(kP, kI, kD, 0.0, 0.0);
    }

    public ClosedLoopConfiguration(final double mV, final double mA) {
        this(0.0, 0.0,0.0, mV, mA);
    }
    
    public ClosedLoopConfiguration(final double kP, final double kI, final double kD, final double mV, final double mA) {
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
        this.mV = mV;
        this.mA = mA;
    }

    public ClosedLoopConfig fillREV(ClosedLoopConfig configuration) {
        configuration.p(kP);
        configuration.i(kI);
        configuration.d(kD);
        return configuration;
    }

    public MAXMotionConfig fillREV(MAXMotionConfig configuration) {
        configuration.maxVelocity(mV);
        configuration.maxAcceleration(mA);
        return configuration;
    }
}
