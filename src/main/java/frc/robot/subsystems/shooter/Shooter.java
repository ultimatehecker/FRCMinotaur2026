package frc.robot.subsystems.shooter;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.minolib.advantagekit.LoggedTracer;
import frc.minolib.advantagekit.LoggedTunableNumber;
import frc.minolib.utilities.SubsystemDataProcessor;
import frc.robot.Robot;
import frc.robot.constants.GlobalConstants;
import frc.robot.constants.ShooterConstants;

import lombok.Getter;
import lombok.RequiredArgsConstructor;
import lombok.experimental.Accessors;

public class Shooter extends SubsystemBase {
    private static final LoggedTunableNumber kP = new LoggedTunableNumber("Shooter/kP");
    private static final LoggedTunableNumber kD = new LoggedTunableNumber("Shooter/kD");
    private static final LoggedTunableNumber kS = new LoggedTunableNumber("Shooter/kS");
    private static final LoggedTunableNumber kV = new LoggedTunableNumber("Shooter/kV");
    private static final LoggedTunableNumber kA = new LoggedTunableNumber("Shooter/kA");

    private static final LoggedTunableNumber kIdleShooterVoltage = new LoggedTunableNumber("Shooter/IdlingVoltage", 2.0);
    private static final LoggedTunableNumber kVelocityTolerance = new LoggedTunableNumber("Shooter/VelocityTolerance", 5.0);

    @RequiredArgsConstructor
    public enum ShooterGoal {
        STOPPED,     
        IDLE,      
        INTERPOLATION, 
        VELOCITY,     
        VOLTAGE;     
    }

    static {
        switch (GlobalConstants.getRobot()) {
            default -> {
                kP.initDefault(ShooterConstants.kP);
                kD.initDefault(ShooterConstants.kD);
                kS.initDefault(ShooterConstants.kS);
                kV.initDefault(ShooterConstants.kV);
                kA.initDefault(ShooterConstants.kA);
            }
            case SIMBOT -> {
                kP.initDefault(ShooterConstants.simulatedKp);
                kD.initDefault(ShooterConstants.simulatedKd);
                kS.initDefault(ShooterConstants.simulatedKs);
                kV.initDefault(ShooterConstants.simulatedKv);
                kA.initDefault(ShooterConstants.simulatedKa);
            }
        }
    }

    private final ShooterIO io;
    private final ShooterIOInputsAutoLogged inputs = new ShooterIOInputsAutoLogged();

    private final Debouncer primaryShooterMotorConnectedDebouncer = new Debouncer(0.5, Debouncer.DebounceType.kFalling);
    private final Alert primaryShooterMotorDisconnectedAlert = new Alert("Primary shooter motor disconnected!", AlertType.kError);

    private final Debouncer secondaryShooterMotorConnectedDebouncer = new Debouncer(0.5, Debouncer.DebounceType.kFalling);
    private final Alert secondaryShooterMotorDisconnectedAlert = new Alert("Secondary shooter motor disconnected!", AlertType.kError);

    private final Debouncer thirdShooterMotorConnectedDebouncer = new Debouncer(0.5, Debouncer.DebounceType.kFalling);
    private final Alert thirdShooterMotorDisconnectedAlert = new Alert("Third shooter motor disconnected!", AlertType.kError);

    @AutoLogOutput(key = "Shooter/BrakeModeEnabled")
    private BooleanSupplier brakeModeEnabled = () -> false;
    private boolean lastBrakeModeEnabled = false;

    @Getter 
    @AutoLogOutput(key = "Shooter/Goal")
    private ShooterGoal goal = ShooterGoal.IDLE;
    
    private double manualVoltage = 0.0;
    private double manualVelocity = 0.0;

    @Getter
    @Accessors(fluent = true)
    @AutoLogOutput(key = "Shooter/AtGoal")
    private boolean atGoal = false;

    public Shooter(ShooterIO io) {
        this.io = io;

        SubsystemDataProcessor.createAndStartSubsystemDataProcessor(() -> {
            synchronized (inputs) {
                io.updateInputs(inputs);
            }
        }, io);
    }

    @Override
    public void periodic() {
        synchronized (inputs) {
            Logger.processInputs("Shooter", inputs);
        }

        // Monitor motor connections
        primaryShooterMotorDisconnectedAlert.set(!primaryShooterMotorConnectedDebouncer.calculate(inputs.isFirstShooterConnected) && !Robot.isJITing());
        secondaryShooterMotorDisconnectedAlert.set(!secondaryShooterMotorConnectedDebouncer.calculate(inputs.isSecondShooterConnected) && !Robot.isJITing());
        thirdShooterMotorDisconnectedAlert.set(!thirdShooterMotorConnectedDebouncer.calculate(inputs.isThirdShooterConnected) && !Robot.isJITing());

        if (kP.hasChanged(hashCode()) || kD.hasChanged(hashCode())) {
            io.setPID(kP.get(), 0.0, kD.get());
        }

        boolean shouldBrake = calculateBrakeMode();
        if (shouldBrake != lastBrakeModeEnabled) {
            io.setBrakeMode(shouldBrake);
            lastBrakeModeEnabled = shouldBrake;
        }

        double targetVelocity = 0.0;
        
        switch (goal) {
            case STOPPED -> {
                io.stop();
                atGoal = true;
            }

            case IDLE -> {
                io.setVoltage(kIdleShooterVoltage.get());
                atGoal = false;  
            }

            case VOLTAGE -> {
                io.setVoltage(manualVoltage);
                atGoal = false; 
            }

            case VELOCITY -> {
                targetVelocity = manualVelocity;
                
                io.setVelocity(
                    targetVelocity, 
                    Math.signum(targetVelocity) * kS.get() + targetVelocity * kV.get()
                );
                
                atGoal = Math.abs(inputs.firstShooterVelocity - targetVelocity) < kVelocityTolerance.get();
            }

            case INTERPOLATION -> {
                targetVelocity = 0.0;
                
                io.setVelocity(
                    targetVelocity, 
                    Math.signum(targetVelocity) * kS.get() + targetVelocity * kV.get()
                );
                
                atGoal = Math.abs(inputs.firstShooterVelocity - targetVelocity) < kVelocityTolerance.get();
            }
        }

        // Logging
        Logger.recordOutput("Shooter/TargetVelocity", targetVelocity);
        Logger.recordOutput("Shooter/ManualVoltage", manualVoltage);
        Logger.recordOutput("Shooter/ManualVelocity", manualVelocity);
        Logger.recordOutput("Shooter/VelocityError", targetVelocity - inputs.firstShooterVelocity);
        
        LoggedTracer.record("ShooterPeriodicMS");
    }

    private boolean calculateBrakeMode() {
        if (brakeModeEnabled.getAsBoolean()) {
            return true;
        }
        
        if (DriverStation.isDisabled()) {
            return false;
        }
        
        return false;
    }

    public void setGoal(ShooterGoal goal) {
        if (goal == this.goal) return;
        this.goal = goal;
        atGoal = false;
    }

    public void setManualVoltage(double voltage) {
        this.manualVoltage = voltage;
    }

    public void setManualVelocity(double velocityRadPerSec) {
        this.manualVelocity = velocityRadPerSec;
    }

    public void setBrakeMode(BooleanSupplier enabled) {
        if (this.brakeModeEnabled.getAsBoolean() == enabled.getAsBoolean()) return;
        this.brakeModeEnabled = enabled;
        io.setBrakeMode(brakeModeEnabled.getAsBoolean());
    }

    public void stop() {
        setGoal(ShooterGoal.STOPPED);
    }

    public double getVelocity() {
        return inputs.firstShooterVelocity;
    }

    public double getVelocityRPM() {
        return inputs.firstShooterVelocity * 60.0 / (2.0 * Math.PI);
    }

    public Command runVelocity(DoubleSupplier velocityRadPerSec) {
        return Commands.startEnd(
            () -> {
                setGoal(ShooterGoal.VELOCITY);
                setManualVelocity(velocityRadPerSec.getAsDouble());
            },
            () -> setGoal(ShooterGoal.STOPPED),
            this
        ).withName("Shooter Run Velocity");
    }

    public Command runVelocity(double velocityRadPerSec) {
        return runVelocity(() -> velocityRadPerSec);
    }

    public Command runVoltage(DoubleSupplier voltage) {
        return Commands.startEnd(
            () -> {
                setGoal(ShooterGoal.VOLTAGE);
                setManualVoltage(voltage.getAsDouble());
            },
            () -> setGoal(ShooterGoal.STOPPED),
            this
        ).withName("Shooter Run Voltage");
    }

    public Command runVoltage(double voltage) {
        return runVoltage(() -> voltage);
    }


    public Command autoAim() {
        return Commands.runOnce(
            () -> setGoal(ShooterGoal.INTERPOLATION),
            this
        ).withName("Shooter Auto Aim");
    }

    public Command idle() {
        return Commands.runOnce(
            () -> setGoal(ShooterGoal.IDLE),
            this
        ).withName("Shooter Idle");
    }

    public Command stopCommand() {
        return Commands.runOnce(
            this::stop,
            this
        ).withName("Shooter Stop");
    }

    public Command waitUntilAtGoal() {
        return Commands.waitUntil(this::atGoal)
            .withTimeout(3.0)
            .withName("Wait For Shooter");
    }

    public double getManualVoltage() {
        return manualVoltage;
    }
}