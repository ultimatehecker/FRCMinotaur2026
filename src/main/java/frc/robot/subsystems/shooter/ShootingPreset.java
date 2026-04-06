package frc.robot.subsystems.shooter;

import frc.minolib.advantagekit.LoggedTunableNumber;

public record ShootingPreset(LoggedTunableNumber hoodAngleDeg, LoggedTunableNumber flywheelSpeed) {}
