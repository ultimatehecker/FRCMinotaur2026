package frc.robot.subsystems.tower;

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
import frc.robot.constants.GlobalConstants;
import frc.robot.constants.TowerConstants;

import lombok.Getter;
import lombok.RequiredArgsConstructor;

public class Tower extends SubsystemBase {
    private static final LoggedTunableNumber topRollerKp = new LoggedTunableNumber("Tower/topRollerKp");
    private static final LoggedTunableNumber topRollerKd = new LoggedTunableNumber("Tower/topRollerKd");
    private static final LoggedTunableNumber topRollerKs = new LoggedTunableNumber("Tower/topRollerKs");
    private static final LoggedTunableNumber topRollerKv = new LoggedTunableNumber("Tower/topRollerKv");
    private static final LoggedTunableNumber topRollerKa = new LoggedTunableNumber("Tower/topRollerKa");

    private static final LoggedTunableNumber bottomRollerKp = new LoggedTunableNumber("Tower/topRollerKp");
    private static final LoggedTunableNumber bottomRollerKd = new LoggedTunableNumber("Tower/topRollerKd");
    private static final LoggedTunableNumber bottomRollerKs = new LoggedTunableNumber("Tower/topRollerKs");
    private static final LoggedTunableNumber bottomRollerKv = new LoggedTunableNumber("Tower/topRollerKv");
    private static final LoggedTunableNumber bottomRollerKa = new LoggedTunableNumber("Tower/topRollerKa");

    @RequiredArgsConstructor
    public enum TowerGoal {
        INTAKE(new LoggedTunableNumber("Tower/IntakeVoltage", 12.0)),
        EXHAUST(new LoggedTunableNumber("Tower/ExhaustVoltage", -6.0)),
        STOP(new LoggedTunableNumber("Tower/StopVoltage", 0.0));

        private final DoubleSupplier voltage;

        public double getVoltage() {
            return voltage.getAsDouble();
        }
    }

    static {
        switch (GlobalConstants.getRobot()) {
            default -> {
                topRollerKp.initDefault(TowerConstants.topRollerkP);
                topRollerKd.initDefault(TowerConstants.topRollerkD);
                topRollerKs.initDefault(TowerConstants.topRollerkS);
                topRollerKv.initDefault(TowerConstants.topRollerkV);
                topRollerKa.initDefault(TowerConstants.topRollerkA);

                bottomRollerKp.initDefault(TowerConstants.bottomRollerkP);
                bottomRollerKd.initDefault(TowerConstants.bottomRollerkD);
                bottomRollerKs.initDefault(TowerConstants.bottomRollerkS);
                bottomRollerKv.initDefault(TowerConstants.bottomRollerkV);
                bottomRollerKa.initDefault(TowerConstants.bottomRollerkA);
            }
            case SIMBOT -> {
                topRollerKp.initDefault(TowerConstants.topRollerkP);
                topRollerKd.initDefault(TowerConstants.topRollerkD);
                topRollerKs.initDefault(TowerConstants.topRollerkS);
                topRollerKv.initDefault(TowerConstants.topRollerkV);
                topRollerKa.initDefault(TowerConstants.topRollerkA);

                bottomRollerKp.initDefault(TowerConstants.simulatedBottomRollerkP);
                bottomRollerKd.initDefault(TowerConstants.simulatedBottomRollerkD);
                bottomRollerKs.initDefault(TowerConstants.simulatedBottomRollerkS);
                bottomRollerKv.initDefault(TowerConstants.simulatedBottomRollerkV);
                bottomRollerKa.initDefault(TowerConstants.simulatedBottomRollerkA);
            }
        }
    }

    private final TowerIO io;
    private final TowerIOInputsAutoLogged inputs = new TowerIOInputsAutoLogged();

    private final Debouncer towerMotorConnectedDebouncer = new Debouncer(0.5, Debouncer.DebounceType.kFalling);
    private final Alert towerMotorDisconnectedAlert = new Alert("Intake pivot motor disconnected!", AlertType.kError);

    @AutoLogOutput(key = "Tower/BrakeModeEnabled")
    private BooleanSupplier brakeModeEnabled = () -> false;

    @Getter 
    private TowerGoal goal = TowerGoal.STOP;
    private double appliedVoltage = 0.0;

    public Tower(TowerIO io) {
        this.io = io;
    }

    @Override
    public void periodic() {
        Logger.processInputs("Tower", inputs);
    
        switch (goal) {
            case INTAKE, EXHAUST -> appliedVoltage = goal.getVoltage();
            case STOP -> appliedVoltage = 0.0;
        }
        
        io.setTopRollerVoltage(appliedVoltage);
        io.setBottomRollerVoltage(appliedVoltage);

        Logger.recordOutput("Tower/AppliedVoltage", appliedVoltage);
        Logger.recordOutput("Tower/WantedVoltage", goal.getVoltage());

        LoggedTracer.record("TowerPeriodicMS");
    }

    public void setGoal(TowerGoal goal) {
        if(goal == this.goal) return;
        this.goal = goal;
    }

    public void setBrakeMode(BooleanSupplier enabled) {
        if (this.brakeModeEnabled.getAsBoolean() == enabled.getAsBoolean()) return;
        this.brakeModeEnabled = enabled;
        io.setBrakeMode(brakeModeEnabled.getAsBoolean());
    }

    public double getTopRollerVelocity() {
        return inputs.topRollerVelocity;
    }

    public double getBottomRollerVelocity() {
        return inputs.topRollerVelocity;
    }
}
