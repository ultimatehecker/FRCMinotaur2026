package frc.robot.subsystems.shooter.flywheel;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import org.ironmaple.simulation.Goal;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.minolib.advantagekit.LoggedTracer;
import frc.minolib.advantagekit.LoggedTunableNumber;
import frc.minolib.math.EqualsUtility;
import frc.minolib.utilities.AllianceFlipUtility;
import frc.minolib.utilities.AllianceFlipUtility;
import frc.minolib.utilities.SubsystemDataProcessor;
import frc.robot.Robot;
import frc.robot.constants.FieldConstants;
import frc.robot.constants.FieldConstants;
import frc.robot.constants.GlobalConstants;
import frc.robot.constants.ShooterConstants;
import frc.robot.subsystems.shooter.ShootingPreset;
import frc.robot.subsystems.shooter.ShootingPreset;
import frc.robot.subsystems.shooter.flywheel.ShooterIO.ShooterIOInputs;

import lombok.Getter;
import lombok.RequiredArgsConstructor;
import lombok.experimental.Accessors;

public class Shooter extends SubsystemBase {
    private static final LoggedTunableNumber kP = new LoggedTunableNumber("Shooter/kP");
    private static final LoggedTunableNumber kD = new LoggedTunableNumber("Shooter/kD");
    private static final LoggedTunableNumber kS = new LoggedTunableNumber("Shooter/kS");
    private static final LoggedTunableNumber kV = new LoggedTunableNumber("Shooter/kV");
    private static final LoggedTunableNumber kA = new LoggedTunableNumber("Shooter/kA");

    private static final LoggedTunableNumber rpmTolerance = new LoggedTunableNumber("Shooter/RPMTolerance", 75.0);
    private static final LoggedTunableNumber readyDebounceSeconds = new LoggedTunableNumber("Shooter/ReadyDebounceSeconds", 0.10);

    private static final ShootingPreset closePreset = new ShootingPreset(
        new LoggedTunableNumber("Shooter/ShootingPreset/ClosePreset/HoodAngleDegrees", 20), 
        new LoggedTunableNumber("Shooter/ShootingPreset/ClosePreset/FlywheelSpeedRPM", 1800), 
        new LoggedTunableNumber("Shooter/ShootingPreset/ClosePreset/VoltageSetpoint", 5.5) 
    );

    private static final ShootingPreset closeMediumPreset = new ShootingPreset(
        new LoggedTunableNumber("Shooter/ShootingPreset/MediumPreset/HoodAngleDegrees", 20), 
        new LoggedTunableNumber("Shooter/ShootingPreset/MediumPreset/FlywheelSpeedRPM", 2250), 
        new LoggedTunableNumber("Shooter/ShootingPreset/MediumPreset/VoltageSetpoint", 6.5) 
    );

    private static final ShootingPreset mediumPreset = new ShootingPreset(
        new LoggedTunableNumber("Shooter/ShootingPreset/MediumPreset/HoodAngleDegrees", 23), 
        new LoggedTunableNumber("Shooter/ShootingPreset/MediumPreset/FlywheelSpeedRPM", 2200), 
        new LoggedTunableNumber("Shooter/ShootingPreset/MediumPreset/VoltageSetpoint", 6.5) 
    );

    private static final ShootingPreset farPreset = new ShootingPreset(
        new LoggedTunableNumber("Shooter/ShootingPreset/FarPreset/HoodAngleDegrees", 27), 
        new LoggedTunableNumber("Shooter/ShootingPreset/FarPreset/FlywheelSpeedRPM", 3300), 
        new LoggedTunableNumber("Shooter/ShootingPreset/FarPreset/VoltageSetpoint", 7.5) 
    );

    public enum ShooterGoal {
        IDLE,
        VELOCITY,
        VOLTAGE,
        REVERSE
    }

    public enum ShooterState {
        IDLE,
        SPINNING_UP,
        AT_VELOCITY,
        AT_VOLTAGE,
        REVERSING
    }

    public enum SelectedShootingPreset {
        CLOSE(closePreset),
        CLOSE_MEDIUM(closeMediumPreset),
        MEDIUM(mediumPreset),
        FAR(farPreset);

        public ShootingPreset shootingPreset;

        private SelectedShootingPreset(ShootingPreset shootingPreset) {
            this.shootingPreset = shootingPreset;
        }

        public ShootingPreset getData() {
            return shootingPreset;
        }
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
                kP.initDefault(0.0);
                kS.initDefault(0.0);
                kV.initDefault(0.0);
                kA.initDefault(0.0);
            }
        }
    }

    private final ShooterIO io;
    private final ShooterIOInputsAutoLogged inputs = new ShooterIOInputsAutoLogged();

    private final Debouncer motorConnectedDebouncer = new Debouncer(0.5, Debouncer.DebounceType.kFalling);
    private final Debouncer readyDebouncer = new Debouncer(readyDebounceSeconds.get(), Debouncer.DebounceType.kFalling);

    private final Alert primaryShooterMotorDisconnectedAlert = new Alert("The primary shooter motor disconnected!", AlertType.kError);
    private final Alert secondaryShooterMotorDisconnectedAlert = new Alert("The secondary shooter motor disconnected!", AlertType.kError);
    private final Alert thirdShooterMotorDisconnectedAlert = new Alert("The third shooter motor disconnected!", AlertType.kError);
    private final Alert fourthShooterMotorDisconnectedAlert = new Alert("The fourth shooter motor disconnected!", AlertType.kError);

    private final Alert primaryShooterMotorTemperatureAlert = new Alert("The primary shooter motor disconnected!", AlertType.kWarning);
    private final Alert secondaryShooterMotorTemperatureAlert = new Alert("The secondary shooter motor disconnected!", AlertType.kWarning);
    private final Alert thirdShooterMotorTemperatureAlert = new Alert("The third shooter motor disconnected!", AlertType.kWarning);
    private final Alert fourthShooterMotorTemperatureAlert = new Alert("The fourth shooter motor disconnected!", AlertType.kWarning);

    @AutoLogOutput(key = "Shooter/BrakeModeEnabled")
    private BooleanSupplier brakeModeEnabled = () -> false;
    private boolean lastBrakeModeEnabled = false;

    @Getter private ShooterGoal goal = ShooterGoal.IDLE;
    @Getter private ShooterState state = ShooterState.IDLE;
    @Getter private SelectedShootingPreset preset = SelectedShootingPreset.CLOSE;

    @Getter @AutoLogOutput(key = "Shooter/VoltageSetpoint") private double voltageSetpoint = 0.0;
    @Getter @AutoLogOutput(key = "Shooter/VelocitySetpointRPM") private double velocitySetpoint = 0.0;

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

        if (kP.hasChanged(hashCode())) {
            io.setPID(kP.get(), 0.0, 0.0);
        }

        primaryShooterMotorDisconnectedAlert.set(!motorConnectedDebouncer.calculate(inputs.isFirstShooterConnected) && !Robot.isJITing());
        secondaryShooterMotorDisconnectedAlert.set(!motorConnectedDebouncer.calculate(inputs.isSecondShooterConnected) && !Robot.isJITing());
        thirdShooterMotorDisconnectedAlert.set(!motorConnectedDebouncer.calculate(inputs.isThirdShooterConnected) && !Robot.isJITing());
        fourthShooterMotorDisconnectedAlert.set(!motorConnectedDebouncer.calculate(inputs.isFourthShooterConnected) && !Robot.isJITing());

        primaryShooterMotorTemperatureAlert.set(!inputs.isFirstShooterConnected);
        secondaryShooterMotorTemperatureAlert.set(!inputs.isSecondShooterConnected);
        thirdShooterMotorTemperatureAlert.set(!inputs.isThirdShooterConnected);
        fourthShooterMotorTemperatureAlert.set(!inputs.isFourthShooterConnected);

        state = handleStateTransition();
        applyState();

        Logger.recordOutput("Shooter/Goal", goal.toString());
        Logger.recordOutput("Shooter/State", state.toString());
        Logger.recordOutput("Shooter/ShootingPreset", preset.toString());

        Logger.recordOutput("Shooter/Goal", goal.toString());
        Logger.recordOutput("Shooter/State", state.toString());
        Logger.recordOutput("Shooter/ShootingPreset", preset.toString());

        LoggedTracer.record("ShooterPeriodic");
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

    private ShooterState handleStateTransition() {
        return switch (goal) {
            case IDLE -> ShooterState.IDLE;
            case VELOCITY -> {
                if (Math.abs(inputs.firstShooterVelocity - velocitySetpoint) <= rpmTolerance.get()) yield ShooterState.AT_VELOCITY;
                yield ShooterState.SPINNING_UP;
            }
            case VOLTAGE -> ShooterState.AT_VOLTAGE;
            case REVERSE -> ShooterState.REVERSING;
        };
    }

    private void applyState() {
        switch (state) {
            case IDLE -> io.stop();
            case SPINNING_UP, AT_VELOCITY -> io.setVelocity(velocitySetpoint, Math.signum(velocitySetpoint) * kS.get() + velocitySetpoint * kV.get());
            case AT_VOLTAGE -> io.setVoltage(voltageSetpoint);
            case REVERSING -> io.setVelocity(velocitySetpoint, 0);
        }
    }

    public void setBrakeMode(BooleanSupplier enabled) {
        io.setBrakeMode(enabled.getAsBoolean());
    }

    public Translation2d getHubCenter(boolean isRed) {
        Translation2d blueHubCenter = FieldConstants.Hub.topCenterPoint.toTranslation2d();

        if (isRed) {
            return new Translation2d(FieldConstants.fieldLength - blueHubCenter.getX(), blueHubCenter.getY());
        }

        return blueHubCenter;
    }

    public void setShootingPreset(SelectedShootingPreset selectedShootingPreset) {
        this.preset = selectedShootingPreset;
    }

    public void runVelocity(double velocityRPM) {
        goal = ShooterGoal.VELOCITY;
        velocitySetpoint = velocityRPM;
    }

    public void runVoltage(double voltage) {
        goal = ShooterGoal.VOLTAGE;
        goal = ShooterGoal.VOLTAGE;
        voltageSetpoint = voltage;
    }

    public void reverse(double rpm) {
        goal = ShooterGoal.REVERSE;
        velocitySetpoint = -Math.abs(rpm);
    }

    public void stop() {
        goal = ShooterGoal.IDLE;
        velocitySetpoint = 0.0;
        readyDebouncer.calculate(false);
    }

    public boolean atSetpoint() {
        return state == ShooterState.AT_VELOCITY;
    }

    @AutoLogOutput(key="Shooter/AtSetpoint")
    public boolean isReady() {
        return readyDebouncer.calculate(atSetpoint());
    }
}