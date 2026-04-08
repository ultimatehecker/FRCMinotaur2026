package frc.robot.subsystems.rollers;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj.Alert;
import frc.minolib.advantagekit.LoggedTracer;
import frc.minolib.utilities.SubsystemDataProcessor;
import frc.robot.Robot;
import lombok.Setter;

public class RollerSystem {
    private final String name;
    private final String inputsName;

    private final RollerSystemIO io;
    protected final RollerSystemIOInputsAutoLogged inputs = new RollerSystemIOInputsAutoLogged();
    
    private final Debouncer motorConnectedDebouncer = new Debouncer(0.5, Debouncer.DebounceType.kFalling);
    private final Alert disconnected;
    private final Alert temperatureFault;

    @Setter private double voltage = 0.0;
    private boolean brakeModeEnabled = true;

    public RollerSystem(String name, String inputsName, RollerSystemIO io) {
        this.name = name;
        this.inputsName = inputsName;
        this.io = io;

        SubsystemDataProcessor.createAndStartSubsystemDataProcessor(() -> {
            synchronized (inputs) {
                io.updateInputs(inputs);
            }
        },
        io);

        disconnected = new Alert("The " + name.toLowerCase() + " motor disconnected!", Alert.AlertType.kError);
        temperatureFault = new Alert("The " + name.toLowerCase() + " motor is overheating!", Alert.AlertType.kWarning);
    }

    public void periodic() {
        synchronized (inputs) {
            Logger.processInputs(inputsName, inputs);
        }
        
        disconnected.set(!motorConnectedDebouncer.calculate(inputs.isMotorConnected) && !Robot.isJITing());
        temperatureFault.set(inputs.temperatureFault);

        // Run roller
        io.setVoltage(voltage);

        // Record cycle time
        LoggedTracer.record(name);
        Logger.recordOutput(inputsName + "/BrakeModeEnabled", brakeModeEnabled);
    }

    public void setBrakeMode(boolean enabled) {
        if (brakeModeEnabled == enabled) return;
        brakeModeEnabled = enabled;

        io.setBrakeMode(enabled);
    }

    public double getTorqueCurrent() {
        return inputs.torqueCurrentAmperes;
    }

    public double getVelocity() {
        return inputs.torqueCurrentAmperes;
    }

    public void stop() {
        voltage = 0.0;
    }
}