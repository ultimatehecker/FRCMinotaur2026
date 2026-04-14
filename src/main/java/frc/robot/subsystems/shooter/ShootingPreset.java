package frc.robot.subsystems.shooter;

import frc.minolib.advantagekit.LoggedTunableNumber;

public record ShootingPreset(LoggedTunableNumber hoodAngleDeg, LoggedTunableNumber flywheelSpeed, LoggedTunableNumber voltageSetpoint) {
    public double getHoodAngleDegrees() {
        return hoodAngleDeg.get();
    }

    public double getFlywheelSpeedRPM() {
        return flywheelSpeed.get();
    }

    public double getVoltageSetpoint() {
        return voltageSetpoint.get();
    }
}
