package frc.robot.subsystems.shooter.flywheel;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.minolib.advantagekit.LoggedTracer;
import frc.minolib.advantagekit.LoggedTunableNumber;
import frc.minolib.utilities.SubsystemDataProcessor;
import frc.robot.Robot;
import frc.robot.constants.GlobalConstants;
import frc.robot.constants.ShooterConstants;
import frc.robot.subsystems.shooter.flywheel.ShooterIO.ShooterIOInputs;
import lombok.Getter;
import lombok.RequiredArgsConstructor;
import lombok.experimental.Accessors;

public class Shooter extends SubsystemBase {
    private static final LoggedTunableNumber kP = new LoggedTunableNumber("Shooter/kP");
    private static final LoggedTunableNumber kS = new LoggedTunableNumber("Shooter/kS");
    private static final LoggedTunableNumber kV = new LoggedTunableNumber("Shooter/kV");
    private static final LoggedTunableNumber kA = new LoggedTunableNumber("Shooter/kA");
    private static final LoggedTunableNumber kMaximumAcceleration = new LoggedTunableNumber("Flywheel/MaxAcceleration", 50.0);

  private SlewRateLimiter slewRateLimiter = new SlewRateLimiter(kMaximumAcceleration.get());

    @RequiredArgsConstructor
    public enum ShooterGoal {
        CLOSE(new LoggedTunableNumber("Shooter/CloseSetpoint", 4)),
        MEDIUM(new LoggedTunableNumber("Shooter/MediumSetpoint", 8)),
        FAR(new LoggedTunableNumber("Shooter/FarSetpoint", 12)),
        IDLE(new LoggedTunableNumber("Shooter/IdleSetpoint", 2)),
        STOP(new LoggedTunableNumber("Shooter/StopSetpoint", 0.0));

        private final DoubleSupplier voltage;

        public double getVoltage() {
            return voltage.getAsDouble();
        }
    }

    static {
        switch (GlobalConstants.getRobot()) {
            default -> {
                kP.initDefault(ShooterConstants.kP);
                kS.initDefault(ShooterConstants.kS);
                kV.initDefault(ShooterConstants.kV);
                kA.initDefault(ShooterConstants.kA);
            }
            case SIMBOT -> {
                kP.initDefault(ShooterConstants.simulatedKp);
                kS.initDefault(ShooterConstants.simulatedKs);
                kV.initDefault(ShooterConstants.simulatedKv);
                kA.initDefault(ShooterConstants.simulatedKa);
            }
        }
    }

    private final ShooterIO io;
    private final ShooterIOInputsAutoLogged inputs = new ShooterIOInputsAutoLogged();

    private final Debouncer primaryShooterMotorConnectedDebouncer = new Debouncer(0.5, Debouncer.DebounceType.kFalling);
    private final Debouncer secondaryShooterMotorConnectedDebouncer = new Debouncer(0.5, Debouncer.DebounceType.kFalling);
    private final Debouncer thirdShooterMotorConnectedDebouncer = new Debouncer(0.5, Debouncer.DebounceType.kFalling);

    private final Alert primaryShooterMotorDisconnectedAlert = new Alert("Primary shooter motor disconnected!", AlertType.kError);
    private final Alert secondaryShooterMotorDisconnectedAlert = new Alert("Secondary shooter motor disconnected!", AlertType.kError);
    private final Alert thirdShooterMotorDisconnectedAlert = new Alert("Third shooter motor disconnected!", AlertType.kError);

    @AutoLogOutput(key = "Shooter/BrakeModeEnabled")
    private BooleanSupplier brakeModeEnabled = () -> false;

    @Getter 
    private ShooterGoal goal = ShooterGoal.IDLE;
    private double appliedVoltage = 0.0;

    @Getter
    @Accessors(fluent = true)
    private boolean manualVoltageOverride = false;

    public Shooter(ShooterIO io) {
        this.io = io;

        SubsystemDataProcessor.createAndStartSubsystemDataProcessor(() -> {
            synchronized (inputs) {
                io.updateInputs(inputs);
            }
        },
        io);
    }

    @Override
    public void periodic() {
        synchronized (inputs) {
            Logger.processInputs("Shooter", inputs);
        }

        primaryShooterMotorDisconnectedAlert.set(!primaryShooterMotorConnectedDebouncer.calculate(inputs.isFirstShooterConnected) && !Robot.isJITing());
        secondaryShooterMotorDisconnectedAlert.set(!secondaryShooterMotorConnectedDebouncer.calculate(inputs.isSecondShooterConnected) && !Robot.isJITing());
        thirdShooterMotorDisconnectedAlert.set(!thirdShooterMotorConnectedDebouncer.calculate(inputs.isThirdShooterConnected) && !Robot.isJITing());
    
        switch (goal) {
            case CLOSE, MEDIUM, FAR, IDLE -> appliedVoltage = goal.getVoltage();
            case STOP -> appliedVoltage = 0.0;
        }
        
        io.setVoltage(appliedVoltage);

        Logger.recordOutput("Shooter/AppliedVoltage", appliedVoltage);
        Logger.recordOutput("Shooter/WantedVoltage", goal.getVoltage());

        LoggedTracer.record("ShooterPeriodicMS");
    }

    public void setGoal(ShooterGoal goal) {
        manualVoltageOverride = false;
        
        if(goal == this.goal) return;
        this.goal = goal;
    }

    public void setBrakeMode(BooleanSupplier enabled) {
        if (this.brakeModeEnabled.getAsBoolean() == enabled.getAsBoolean()) return;
        this.brakeModeEnabled = enabled;
        io.setBrakeMode(brakeModeEnabled.getAsBoolean());
    }

    public double getVelocity() {
        return inputs.firstShooterVelocity;
    }

    public void setManualVoltage(double voltage) {
        manualVoltageOverride = true;
        io.setVoltage(voltage);
    }
}
