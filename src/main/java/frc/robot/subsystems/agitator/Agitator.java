package frc.robot.subsystems.agitator;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.minolib.advantagekit.LoggedTracer;
import frc.minolib.advantagekit.LoggedTunableNumber;
import frc.minolib.utilities.SubsystemDataProcessor;
import frc.robot.constants.AgitatorConstants;
import frc.robot.constants.GlobalConstants;

import lombok.Getter;
import lombok.RequiredArgsConstructor;

public class Agitator extends SubsystemBase {
    private static final LoggedTunableNumber kP = new LoggedTunableNumber("Agitator/kP");
    private static final LoggedTunableNumber kD = new LoggedTunableNumber("Agitator/kD");
    private static final LoggedTunableNumber kS = new LoggedTunableNumber("Agitator/kS");
    private static final LoggedTunableNumber kV = new LoggedTunableNumber("Agitator/kV");
    private static final LoggedTunableNumber kA = new LoggedTunableNumber("Agitator/kA");

    @RequiredArgsConstructor
    public enum AgitatorGoal {
        FEED(new LoggedTunableNumber("Agitator/IntakeVoltage", 12.0)),
        EXHAUST(new LoggedTunableNumber("Agitator/ExhaustVoltage", -6.0)),
        STOP(new LoggedTunableNumber("Agitator/StopVoltage", 0.0));

        private final DoubleSupplier voltage;

        public double getVoltage() {
            return voltage.getAsDouble();
        }
    }

    static {
        switch (GlobalConstants.getRobot()) {
            default -> {
                kP.initDefault(AgitatorConstants.kP);
                kD.initDefault(AgitatorConstants.kD);
                kS.initDefault(AgitatorConstants.kD);
                kV.initDefault(AgitatorConstants.kV);
                kA.initDefault(AgitatorConstants.kA);
            }
            case SIMBOT -> {
                kP.initDefault(AgitatorConstants.simulatedKp);
                kD.initDefault(AgitatorConstants.simulatedKi);
                kS.initDefault(AgitatorConstants.simulatedKs);
                kV.initDefault(AgitatorConstants.simulatedKv);
                kA.initDefault(AgitatorConstants.simulatedKa);
            }
        }
    }

    private final AgitatorIO io;
    private final AgitatorIOInputsAutoLogged inputs = new AgitatorIOInputsAutoLogged();

    private final Debouncer indexerMotorConnectedDebouncer = new Debouncer(0.5, Debouncer.DebounceType.kFalling);
    private final Alert indexerMotorDisconnectedAlert = new Alert("Intake pivot motor disconnected!", AlertType.kError);

    @AutoLogOutput(key = "Agitator/BrakeModeEnabled")
    private BooleanSupplier brakeModeEnabled = () -> false;

    @Getter 
    private AgitatorGoal goal = AgitatorGoal.STOP;
    private double rollerVoltage = 0.0;

    public Agitator(AgitatorIO io) {
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
        synchronized(inputs) {
            Logger.processInputs("Agitator", inputs);
        }

        switch (goal) {
            case FEED, EXHAUST -> rollerVoltage = goal.getVoltage();
            case STOP -> rollerVoltage = 0.0;
        }
        
        io.setVoltage(rollerVoltage);

        Logger.recordOutput("Agitator/AppliedVoltage", rollerVoltage);
        Logger.recordOutput("Agitator/WantedVoltage", goal.getVoltage());

        LoggedTracer.record("AgitatorPeriodicMS");
    }

    public void setGoal(AgitatorGoal goal) {
        if(goal == this.goal) return;
        this.goal = goal;
    }

    public void setBrakeMode(BooleanSupplier enabled) {
        if (this.brakeModeEnabled.getAsBoolean() == enabled.getAsBoolean()) return;
        this.brakeModeEnabled = enabled;
        io.setBrakeMode(brakeModeEnabled.getAsBoolean());
    }

    public double getVelocity() {
        return inputs.leftVelocity;
    }
}
