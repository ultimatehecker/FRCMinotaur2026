package frc.robot.subsystems.shooter.hood;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Radians;

import java.util.function.BooleanSupplier;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.minolib.advantagekit.LoggedTracer;
import frc.minolib.advantagekit.LoggedTunableNumber;
import frc.minolib.math.EqualsUtility;
import frc.robot.Robot;
import frc.robot.constants.GlobalConstants;
import frc.robot.constants.HoodConstants;

import lombok.Getter;

public class Hood extends SubsystemBase {
    private static final LoggedTunableNumber kP = new LoggedTunableNumber("Hood/Gains/kP");
    private static final LoggedTunableNumber kD = new LoggedTunableNumber("Hood/Gains/kD");
    private static final LoggedTunableNumber kS = new LoggedTunableNumber("Hood/Gains/kS");
    private static final LoggedTunableNumber kV = new LoggedTunableNumber("Hood/Gains/kV");
    private static final LoggedTunableNumber kA = new LoggedTunableNumber("Hood/Gains/kA");

    private static final LoggedTunableNumber kMinimumAngleDegrees = new LoggedTunableNumber("Hood/MinimumAngleDegrees", HoodConstants.kHoodMinimumPosition.in(Degrees));
    private static final LoggedTunableNumber kMaximumAngleDegrees = new LoggedTunableNumber("Hood/MaximumAngleDegrees", HoodConstants.kHoodMaximumPosition.in(Degrees));
    
    private static final LoggedTunableNumber kHomingVoltage = new LoggedTunableNumber("Hood/Homing/Voltage", -2);
    private static final LoggedTunableNumber kHomingVelocityThreshold = new LoggedTunableNumber("Hood/Homing/VelocityThreshold", 0.05);
    private static final LoggedTunableNumber kHomingTimeoutSeconds = new LoggedTunableNumber("Hood/Homing/TimeoutSeconds", 0.4);

    private static final LoggedTunableNumber toleranceDegrees = new LoggedTunableNumber("Hood/ToleranceDegrees", 1.0);
    private static final LoggedTunableNumber readyDebounceSeconds = new LoggedTunableNumber("Hood/ReadyDebounceSeconds", 0.08);

    static {
        switch (GlobalConstants.getRobot()) {
            default -> {
                kP.initDefault(HoodConstants.kP);
                kD.initDefault(HoodConstants.kD);
                kS.initDefault(HoodConstants.kS);
                kV.initDefault(HoodConstants.kV);
                kA.initDefault(HoodConstants.kA);
            }

            case SIMBOT -> {
                kP.initDefault(5.0);
                kD.initDefault(0.0);
                kS.initDefault(0.0);
                kV.initDefault(0.0);
                kA.initDefault(0.0);
            }
        }
    }

    private final HoodIO io;
    private final HoodIOInputsAutoLogged inputs = new HoodIOInputsAutoLogged();

    private final Debouncer motorConnectedDebouncer = new Debouncer(0.5, Debouncer.DebounceType.kFalling);
    private Debouncer homingDebouncer = new Debouncer(kHomingTimeoutSeconds.get(), Debouncer.DebounceType.kRising);
    private Debouncer readyDebouncer = new Debouncer(readyDebounceSeconds.get(), Debouncer.DebounceType.kFalling);

    private final Alert motorDisconnectedAlert = new Alert("The hood motor is disconnected!", AlertType.kError);
    private final Alert motorOverheatingAlert = new Alert("The hood motor is overheating!", AlertType.kWarning);

    private BooleanSupplier coastOverride = () -> false;
    private BooleanSupplier disabledOverride = () -> false;

    @AutoLogOutput(key = "Hood/BrakeModeEnabled") 
    private boolean brakeModeEnabled = false;

    @Getter private boolean zeroed = false;
    private double targetAngleRadians = HoodConstants.kHoodMinimumPosition.in(Radians);

    private final Command zeroCommand;

    public Hood(HoodIO io) {
        this.io = io;

        zeroCommand = zeroCommand();
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Hood", inputs);   

        motorDisconnectedAlert.set(!motorConnectedDebouncer.calculate(inputs.isMotorConnected) && !Robot.isJITing());
        motorOverheatingAlert.set(inputs.temperatureFault);

        Robot.batteryLogger.reportCurrentUsage("Hood", inputs.isMotorConnected ? inputs.supplyCurrentAmperes : 0.0);

        // Update tunable numbers
        if (kP.hasChanged(hashCode()) || kD.hasChanged(hashCode()) || kS.hasChanged(hashCode()) || kV.hasChanged(hashCode()) || kA.hasChanged(hashCode())) {
            io.setPID(kP.get(), 0.0, kD.get(), kS.get(), kV.get(), kA.get());
        }

        if (readyDebounceSeconds.hasChanged(hashCode())) {
            readyDebouncer.setDebounceTime(readyDebounceSeconds.get());
        }

        if (kHomingTimeoutSeconds.hasChanged(hashCode())) {
            homingDebouncer.setDebounceTime(kHomingTimeoutSeconds.get());
        }

        if (DriverStation.isEnabled() && !zeroed && !zeroCommand.isScheduled()) {
            CommandScheduler.getInstance().schedule(zeroCommand);
        }

        setBrakeMode(!coastOverride.getAsBoolean()); 

        if (!Double.isNaN(targetAngleRadians) && zeroed) {
            double restrictedSetpoint = MathUtil.clamp(
                targetAngleRadians,
                Units.degreesToRadians(kMinimumAngleDegrees.get()),
                Units.degreesToRadians(kMaximumAngleDegrees.get())
            );

            io.setPosition(restrictedSetpoint);
        }

        Logger.recordOutput("Hood/Zeroed", zeroed);
        Logger.recordOutput("Hood/TargetAngleDegrees", Units.radiansToDegrees(targetAngleRadians));
        Logger.recordOutput("Hood/AtTarget", atTarget());
        Logger.recordOutput("Hood/IsReady", isReady());
        Logger.recordOutput("Hood/CoastOverride", coastOverride.getAsBoolean());
        Logger.recordOutput("Hood/DisabledOverride", disabledOverride.getAsBoolean());
        
        SmartDashboard.putBoolean("Hood At Minimum Angle", EqualsUtility.epsilonEquals(inputs.positionRadians, Units.degreesToRadians(kMinimumAngleDegrees.get())));
        
        LoggedTracer.record("HoodPeriodic");
    }

    @AutoLogOutput(key = "Hood/MeasuredAngleDegrees")
    public double getMeasuredAngleDegrees() {
        return Units.radiansToDegrees(inputs.positionRadians);
    }

    public void setAngleDegrees(double degrees) {
        targetAngleRadians = Units.degreesToRadians(degrees);
    }

    public void setOverrides(BooleanSupplier coastOverride, BooleanSupplier disabledOverride) {
        this.coastOverride = coastOverride;
        this.disabledOverride = disabledOverride;
    }

    private void setBrakeMode(boolean enabled) {
        if (brakeModeEnabled == enabled) return;
        brakeModeEnabled = enabled;
        io.setBrakeMode(brakeModeEnabled);
    }

    public void stow() {
        targetAngleRadians = Units.degreesToRadians(kMinimumAngleDegrees.get());
    }

    public void stop() {
        targetAngleRadians = Double.NaN;
        readyDebouncer.calculate(false);
        io.stop();
    }

    @AutoLogOutput(key = "Hood/AtTarget")
    public boolean atTarget() {
        if (Double.isNaN(targetAngleRadians)) return false;
        return Math.abs(inputs.positionRadians - targetAngleRadians) <= Units.degreesToRadians(toleranceDegrees.get());
    }

    @AutoLogOutput(key = "Hood/IsReady")
    public boolean isReady() {
        return zeroed && readyDebouncer.calculate(atTarget());
    }

    public Command stowCommand() {
        return run(this::stow).withName("Stow Hood");
    }

    public Command zeroCommand() {
        return Commands.sequence(
            Commands.runOnce(() -> {
                zeroed = false;
                homingDebouncer = new Debouncer(kHomingTimeoutSeconds.get(), Debouncer.DebounceType.kRising);
                homingDebouncer.calculate(false);
                targetAngleRadians = Double.NaN;
            }),
            Commands.run(() -> {
                io.setVoltage(kHomingVoltage.get());
            }).until(() -> {
                    return homingDebouncer.calculate(Math.abs(inputs.velocityRadiansPerSecond) <= kHomingVelocityThreshold.get() && Math.abs(inputs.appliedVoltage) >= Math.abs(kHomingVoltage.get()) * 0.7);
            }).withTimeout(3.0),
            Commands.runOnce(() -> {
                io.resetPosition();

                zeroed = true;
                stow();
            })
        )
        .withName("Zero Hood")
        .ignoringDisable(false);
    }
}
