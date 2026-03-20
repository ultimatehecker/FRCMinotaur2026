package frc.robot.subsystems.hang;

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
import frc.robot.constants.HangConstants;

import lombok.Getter;
import lombok.RequiredArgsConstructor;

public class Hang extends SubsystemBase {
    private static final LoggedTunableNumber pivotKp = new LoggedTunableNumber("Hang/Pivot/kP");
    private static final LoggedTunableNumber pivotKd = new LoggedTunableNumber("Hang/Pivot/kD");
    private static final LoggedTunableNumber pivotKs = new LoggedTunableNumber("Hang/Pivot/kS");
    private static final LoggedTunableNumber pivotKv = new LoggedTunableNumber("Hang/Pivot/kV");
    private static final LoggedTunableNumber pivotKa = new LoggedTunableNumber("Hang/Pivot/kA");

    private static final LoggedTunableNumber winderKp = new LoggedTunableNumber("Hang/Winder/kP");
    private static final LoggedTunableNumber winderKd = new LoggedTunableNumber("Hang/Winder/kD");
    private static final LoggedTunableNumber winderKs = new LoggedTunableNumber("Hang/Winder/kS");
    private static final LoggedTunableNumber winderKv = new LoggedTunableNumber("Hang/Winder/kV");
    private static final LoggedTunableNumber winderKa = new LoggedTunableNumber("Hang/Winder/kA");

    @RequiredArgsConstructor
    public enum HangPivotGoal {
        UP(new LoggedTunableNumber("Hang/Pivot/UpVoltage", 12.0)),
        DOWN(new LoggedTunableNumber("Hang/Pivot/DownVoltage", -12.0)),
        STOP(new LoggedTunableNumber("Hang/Pivot/StopVoltage", 0.0));

        private final DoubleSupplier rollerVoltage;

        public double getRollerVoltage() {
            return rollerVoltage.getAsDouble();
        }
    }

    @RequiredArgsConstructor
    public enum WinderGoal {
        UP(new LoggedTunableNumber("Hang/Winder/UpVoltage", 4.0)),
        DOWN(new LoggedTunableNumber("Hang/Winder/DownVoltage", -4.0)),
        STOP(new LoggedTunableNumber("Hang/Winder/StopVoltage", 0.0));

        private final DoubleSupplier rollerVoltage;

        public double getRollerVoltage() {
            return rollerVoltage.getAsDouble();
        }
    }

    static {
        switch (GlobalConstants.getRobot()) {
            default -> {
                pivotKp.initDefault(HangConstants.pivotKp);
                pivotKd.initDefault(HangConstants.pivotKd);
                pivotKs.initDefault(HangConstants.pivotKs);
                pivotKv.initDefault(HangConstants.pivotKv);
                pivotKa.initDefault(HangConstants.pivotKa);

                winderKp.initDefault(HangConstants.winderkP);
                winderKd.initDefault(HangConstants.winderkD);
                winderKs.initDefault(HangConstants.winderkS);
                winderKv.initDefault(HangConstants.winderkV);
                winderKa.initDefault(HangConstants.winderkA);
            }
            case SIMBOT -> {
                pivotKp.initDefault(HangConstants.simulatedPivotKp);
                pivotKd.initDefault(HangConstants.simulatedPivotKd);
                pivotKs.initDefault(HangConstants.simulatedPivotKs);
                pivotKv.initDefault(HangConstants.simulatedPivotKv);
                pivotKa.initDefault(HangConstants.simulatedPivotKa);

                winderKp.initDefault(HangConstants.simulatedWinderKp);
                winderKd.initDefault(HangConstants.simulatedWinderKd);
                winderKs.initDefault(HangConstants.simulatedWinderKs);
                winderKv.initDefault(HangConstants.simulatedWinderKv);
                winderKa.initDefault(HangConstants.simulatedWinderKa);
            }
        }
    }

    private final HangIO io;
    private final HangIOInputsAutoLogged inputs = new HangIOInputsAutoLogged();

    private final Debouncer pivotMotorConnectedDebouncer = new Debouncer(0.5, Debouncer.DebounceType.kFalling);
    private final Debouncer winderMotorConnectedDebouncer = new Debouncer(0.5, Debouncer.DebounceType.kFalling);

    private final Alert pivotMotorDisconnectedAlert = new Alert("Hang pivot motor disconnected!", AlertType.kError);
    private final Alert winderMotorDisconnectedAlert = new Alert("Hang winder motor disconnected!", AlertType.kError);

    @AutoLogOutput(key = "Hang/BrakeModeEnabled")
    private BooleanSupplier brakeModeEnabled = () -> false;

    @Getter 
    private HangPivotGoal pivotGoal = HangPivotGoal.STOP;
    private double pivotVoltage = 0.0;

    @Getter 
    private WinderGoal winderGoal = WinderGoal.STOP;
    private double winderVoltage = 0.0;

    public Hang(HangIO io) {
        this.io = io;
    }

    @Override
    public void periodic() {
        synchronized (inputs) {
            Logger.processInputs("Hang", inputs);
        }
    
        switch (pivotGoal) {
            case UP, DOWN -> pivotVoltage = pivotGoal.getRollerVoltage();
            case STOP -> pivotVoltage = 0.0;
        }
        
        io.setPivotVoltage(pivotVoltage);

        switch (winderGoal) {
            case UP, DOWN -> winderVoltage = winderGoal.getRollerVoltage();
            case STOP -> winderVoltage = 0.0;
        }
        
        io.setWinderVoltage(winderVoltage);

        Logger.recordOutput("Hang/Pivot/AppliedVoltage", pivotVoltage);
        Logger.recordOutput("Hang/Pivot/WantedVoltage", pivotGoal.getRollerVoltage());

        Logger.recordOutput("Hang/Winder/AppliedVoltage", winderVoltage);
        Logger.recordOutput("Hang/Winder/WantedVoltage", winderGoal.getRollerVoltage());

        LoggedTracer.record("HangPeriodicMS");
    }

    public void setPivotGoal(HangPivotGoal goal) {
        if(goal == this.pivotGoal) return;
        this.pivotGoal = goal;
    }

    public void setWinderGoal(WinderGoal goal) {
        if(goal == this.winderGoal) return;
        this.winderGoal = goal;
    }

    public void setBrakeMode(BooleanSupplier enabled) {
        if (this.brakeModeEnabled.getAsBoolean() == enabled.getAsBoolean()) return;
        this.brakeModeEnabled = enabled;
        io.setBrakeMode(brakeModeEnabled.getAsBoolean());
    }

    public double getPivotVelocity() {
        return inputs.pivotVelocity;
    }
}
