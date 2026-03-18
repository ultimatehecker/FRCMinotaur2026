package frc.robot.subsystems.indexer;

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
import frc.robot.constants.GlobalConstants;
import frc.robot.constants.IndexerConstants;

import lombok.Getter;
import lombok.RequiredArgsConstructor;

public class Indexer extends SubsystemBase {
    private static final LoggedTunableNumber kP = new LoggedTunableNumber("Intake/kP");
    private static final LoggedTunableNumber kD = new LoggedTunableNumber("Intake/kD");
    private static final LoggedTunableNumber kS = new LoggedTunableNumber("Intake/kS");
    private static final LoggedTunableNumber kV = new LoggedTunableNumber("Intake/kV");
    private static final LoggedTunableNumber kA = new LoggedTunableNumber("Intake/kA");

    @RequiredArgsConstructor
    public enum Goal {
        INTAKE(new LoggedTunableNumber("Indexer/IntakeVoltage", 12.0)),
        EXHAUST(new LoggedTunableNumber("Indexer/ExhaustVoltage", -6.0)),
        IDLE(new LoggedTunableNumber("Indexer/IdleVoltage", 2.0)),
        STOP(new LoggedTunableNumber("Indexer/StopVoltage", 0.0));

        private final DoubleSupplier rollerVoltage;

        public double getRollerVoltage() {
            return rollerVoltage.getAsDouble();
        }
    }

    static {
        switch (GlobalConstants.getRobot()) {
            default -> {
                kP.initDefault(IndexerConstants.kP);
                kD.initDefault(IndexerConstants.kD);
                kS.initDefault(IndexerConstants.kD);
                kV.initDefault(IndexerConstants.kV);
                kA.initDefault(IndexerConstants.kA);
            }
            case SIMBOT -> {
                kP.initDefault(IndexerConstants.simulatedKp);
                kD.initDefault(IndexerConstants.simulatedKi);
                kS.initDefault(IndexerConstants.simulatedKs);
                kV.initDefault(IndexerConstants.simulatedKv);
                kA.initDefault(IndexerConstants.simulatedKa);
            }
        }
    }

    private final IndexerIO io;
    private final IndexerIOInputsAutoLogged inputs = new IndexerIOInputsAutoLogged();

    private final Debouncer indexerMotorConnectedDebouncer = new Debouncer(0.5, Debouncer.DebounceType.kFalling);
    private final Alert indexerMotorDisconnectedAlert = new Alert("Intake pivot motor disconnected!", AlertType.kError);

    @AutoLogOutput(key = "Indexer/BrakeModeEnabled")
    private BooleanSupplier brakeModeEnabled = () -> false;

    @Getter 
    private Goal goal = Goal.STOP;
    private double rollerVoltage = 0.0;

    public Indexer(IndexerIO io) {
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
            Logger.processInputs("Indexer", inputs);
        }
    
        switch (goal) {
            case INTAKE, EXHAUST, IDLE -> rollerVoltage = goal.getRollerVoltage();
            case STOP -> rollerVoltage = 0.0;
        }
        
        io.setVoltage(rollerVoltage);

        Logger.recordOutput("Indexer/AppliedVoltage", rollerVoltage);
        Logger.recordOutput("Indexer/WantedVoltage", goal.getRollerVoltage());

        LoggedTracer.record("IndexerPeriodicMS");
    }

    public void setGoal(Goal goal) {
        if(goal == this.goal) return;
        this.goal = goal;
    }

    public void setBrakeMode(BooleanSupplier enabled) {
        if (this.brakeModeEnabled.getAsBoolean() == enabled.getAsBoolean()) return;
        this.brakeModeEnabled = enabled;
        io.setBrakeMode(brakeModeEnabled.getAsBoolean());
    }

    public double getVelocity() {
        return inputs.velocity;
    }
}
