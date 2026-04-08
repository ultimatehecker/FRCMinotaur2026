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
import frc.robot.subsystems.rollers.RollerSystem;
import frc.robot.subsystems.rollers.RollerSystemIO;

import lombok.Getter;
import lombok.RequiredArgsConstructor;
import lombok.Setter;

public class Tower extends SubsystemBase {
    private static final LoggedTunableNumber topkP = new LoggedTunableNumber("Tower/Top/kP");
    private static final LoggedTunableNumber topkD = new LoggedTunableNumber("Tower/Top/kD");
    private static final LoggedTunableNumber topkS = new LoggedTunableNumber("Tower/Top/kS");
    private static final LoggedTunableNumber topkV = new LoggedTunableNumber("Tower/Top/kV");

    private static final LoggedTunableNumber bottomkP = new LoggedTunableNumber("Tower/Bottom/kP");
    private static final LoggedTunableNumber bottomkD = new LoggedTunableNumber("Tower/Bottom/kD");
    private static final LoggedTunableNumber bottomkS = new LoggedTunableNumber("Tower/Bottom/kS");
    private static final LoggedTunableNumber bottomkV = new LoggedTunableNumber("Tower/Bottom/kV");

    @RequiredArgsConstructor
    public enum TowerGoal {
        FEED(new LoggedTunableNumber("Tower/FeedVoltage", 12.0)),
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
                topkP.initDefault(TowerConstants.topkP);
                topkD.initDefault(TowerConstants.topkD);
                topkS.initDefault(TowerConstants.topkS);
                topkV.initDefault(TowerConstants.topkV);

                bottomkP.initDefault(TowerConstants.bottomkP);
                bottomkD.initDefault(TowerConstants.bottomkD);
                bottomkS.initDefault(TowerConstants.bottomkS);
                bottomkV.initDefault(TowerConstants.bottomkV);
            }
            case SIMBOT -> {
                topkP.initDefault(0.0);
                topkD.initDefault(0.0);
                topkS.initDefault(0.0);
                topkV.initDefault(0.0);

                bottomkP.initDefault(0.0);
                bottomkD.initDefault(0.0);
                bottomkS.initDefault(0.0);
                bottomkV.initDefault(0.0);
            }
        }
    }

    private final RollerSystem topRoller;
    private final RollerSystem bottomRoller;

    @Getter @Setter @AutoLogOutput private TowerGoal towerGoal = TowerGoal.STOP;

    public Tower(final RollerSystemIO topRollerIO, final RollerSystemIO bottomRollerIO) {
        topRoller = new RollerSystem("Tower Top Roller", "Tower/TopRoller", topRollerIO);
        bottomRoller = new RollerSystem("Tower Bottom Roller", "Tower/BottomRoller", bottomRollerIO);
    }

    @Override
    public void periodic() {
        topRoller.periodic();
        bottomRoller.periodic();
    
        double appliedVoltage = 0.0;

        switch (towerGoal) {
            case FEED, EXHAUST -> appliedVoltage = towerGoal.getVoltage();
            case STOP -> appliedVoltage = 0.0;
        }
        
        topRoller.setVoltage(appliedVoltage);
        bottomRoller.setVoltage(appliedVoltage);

        LoggedTracer.record("TowerPeriodic");
    }
}
