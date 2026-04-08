package frc.robot.io;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public interface DriverControllerIO {
    public double getThrottle();

    public double getStrafe();

    public double getRotation();

    public double getRotationY();

    public Trigger resetGyro();

    public Trigger deployIntake();

    public Trigger stowIntake();

    public Trigger automaticallyShoot();

    public XboxController getPrimaryHID();

    public void rumble(boolean intensity);
}