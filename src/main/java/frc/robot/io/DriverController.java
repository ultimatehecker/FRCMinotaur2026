package frc.robot.io;

import com.google.flatbuffers.Constants;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.minolib.controller.CommandSimulatedXboxController;
import frc.robot.Robot;
import frc.robot.constants.ControllerConstants;

public class DriverController implements DriverControllerIO {
    private static DriverController instance = null;

    public static DriverController getInstance() {
        if (instance == null) {
            instance = new DriverController();
        }

        return instance;
    }

    private final CommandXboxController controller;

    private DriverController() {
        if (Robot.isSimulation()) {
            controller = new CommandSimulatedXboxController(ControllerConstants.kDriverControllerPort);
        } else {
            controller = new CommandXboxController(ControllerConstants.kDriverControllerPort);
        }
    }

    @Override
    public double getThrottle() {
        return -(Math.pow(Math.abs(controller.getLeftY()), 1.5)) * Math.signum(controller.getLeftY());
    }

    @Override
    public double getStrafe() {
        return -(Math.pow(Math.abs(controller.getLeftX()), 1.5)) * Math.signum(controller.getLeftX());
    }

    @Override
    public double getRotation() {
        return -(Math.pow(Math.abs(controller.getRightX()), 2.0)) * Math.signum(controller.getRightX());
    }

    @Override
    public double getRotationY() {
        return -(Math.pow(Math.abs(controller.getRightY()), 2.0)) * Math.signum(controller.getRightY());
    }

    @Override
    public Trigger resetGyro() {
        return controller.back().and(controller.start().negate());
    }

    @Override
    public Trigger deployIntake() {
        return controller.leftTrigger();
    }

    @Override
    public Trigger stowIntake() {
        return controller.leftBumper();
    }

    @Override
    public Trigger automaticallyShoot() {
        return controller.rightTrigger();
    }

    @Override
    public Trigger automaticallyAim() {
        return controller.rightBumper();
    }

    @Override
    public Trigger automaticallyHang() {
        return controller.povLeft();
    }

    @Override
    public XboxController getPrimaryHID() {
        return controller.getHID();
    }

    @Override
    public void rumble(boolean rumble) {
        controller.getHID().setRumble(RumbleType.kBothRumble, rumble ? 1 : 0);
    }
}
