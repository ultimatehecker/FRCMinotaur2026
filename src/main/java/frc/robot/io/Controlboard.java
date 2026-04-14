package frc.robot.io;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class Controlboard implements DriverControllerIO, OperatorControllerIO {
    private static Controlboard instance = null;

    public static Controlboard getInstance() {
        if (instance == null) {
            instance = new Controlboard();
        }
        return instance;
    }

    private final DriverControllerIO driverController;
    private final OperatorControllerIO operatorController;

    private Controlboard() {
        driverController = DriverController.getInstance();
        operatorController = OperatorController.getInstance();

    }

    @Override
    public double getThrottle() {
        return driverController.getThrottle();
    }

    @Override
    public double getStrafe() {
        return driverController.getStrafe();
    }

    @Override
    public double getRotation() {
        return driverController.getRotation();
    }

    @Override
    public double getRotationY() {
        return driverController.getRotationY();
    }

    @Override
    public Trigger resetGyro() {
        return driverController.resetGyro();
    }

    @Override
    public Trigger deployIntake() {
        return driverController.deployIntake();
    }

    @Override
    public Trigger exhaust() {
        return driverController.exhaust();
    }

    @Override
    public Trigger stowIntake() {
        return driverController.stowIntake();
    }

    @Override
    public Trigger rollIndexer() {
        return driverController.rollIndexer();
    }

    @Override
    public Trigger automaticallyShoot() {
        return driverController.automaticallyShoot();
    }

    @Override
    public Trigger automaticallyAim() {
        return driverController.automaticallyAim();
    }

    @Override
    public Trigger automaticallyHang() {
        return driverController.automaticallyHang();
    }

    @Override
    public XboxController getPrimaryHID() {
        return driverController.getPrimaryHID();
    }

    @Override
    public XboxController getSecondaryHID() {
        return operatorController.getSecondaryHID();
    }

    @Override
    public Trigger selectCloseShootingPreset() {
        return operatorController.selectCloseShootingPreset();
    } 

    @Override
    public Trigger selectMediumShootingPreset() {
        return operatorController.selectMediumShootingPreset();
    }

    @Override
    public Trigger selectFarShootingPreset() {
        return operatorController.selectFarShootingPreset();
    }

    @Override
    public Trigger deployClimber() {
        return operatorController.deployClimber();
    }

    @Override
    public Trigger stowClimber() {
        return operatorController.stowClimber();
    }

    @Override
    public void rumble(boolean intensity) {
        driverController.rumble(intensity);
    }
}

