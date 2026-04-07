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
import frc.robot.subsystems.rollers.RollerSystem;
import frc.robot.subsystems.rollers.RollerSystemIO;
import lombok.Getter;
import lombok.RequiredArgsConstructor;
import lombok.Setter;

public class Indexer extends SubsystemBase {
    private static final LoggedTunableNumber kP = new LoggedTunableNumber("Indexer/kP");
    private static final LoggedTunableNumber kD = new LoggedTunableNumber("Indexer/kD");
    private static final LoggedTunableNumber kS = new LoggedTunableNumber("Indexer/kS");
    private static final LoggedTunableNumber kV = new LoggedTunableNumber("Indexer/kV");
    private static final LoggedTunableNumber kA = new LoggedTunableNumber("Indexer/kA");

    @RequiredArgsConstructor
    public enum IndexerGoal {
        STOP(new LoggedTunableNumber("Indexer/StopVoltage", 0.0)),
        FEED(new LoggedTunableNumber("Indexer/IntakeVoltage", 12.0)),
        EXHAUST(new LoggedTunableNumber("Indexer/ExhaustVoltage", -6.0)),
        IDLE(new LoggedTunableNumber("Indexer/IdleVoltage", 1.5));

        private final DoubleSupplier voltage;

        public double getVoltage() {
            return voltage.getAsDouble();
        }
    }

    static {
        switch (GlobalConstants.getRobot()) {
            default -> {
                kP.initDefault(0.0);
                kD.initDefault(0.0);
                kS.initDefault(0.0);
                kV.initDefault(0.0);
                kA.initDefault(0.0);
            }
            case SIMBOT -> {
                kP.initDefault(0.0);
                kD.initDefault(0.0);
                kS.initDefault(0.0);
                kV.initDefault(0.0);
                kA.initDefault(0.0);
            }
        }
    }

    private final RollerSystem leftRoller;
    private final RollerSystem rightRoller;
    @Getter @Setter @AutoLogOutput private IndexerGoal goal = IndexerGoal.STOP;

    public Indexer(RollerSystemIO leftRollerIO, RollerSystemIO rightRollerIO) {
        leftRoller = new RollerSystem("Indexer", "Indexer/LeftRoller", leftRollerIO);
        rightRoller = new RollerSystem("Indexer", "Indexer/RightRoller", rightRollerIO);
    }

    @Override
    public void periodic() {
        leftRoller.periodic();
        rightRoller.periodic();

        double rollerVoltage = 0.0;

        switch(goal) {
            case FEED, EXHAUST, IDLE -> {
                rollerVoltage = goal.getVoltage();
            }

            case STOP -> {
                rollerVoltage = 0.0;
            }
        }

        leftRoller.setVoltage(rollerVoltage);
        rightRoller.setVoltage(rollerVoltage);
    }
}
