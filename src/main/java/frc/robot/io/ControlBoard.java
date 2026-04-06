package frc.robot.io;

import edu.wpi.first.wpilibj2.command.button.Trigger;

public class ControlBoard implements DriverControllerIO, OperatorControllerIO {
    private static ControlBoard instance = null;

    public static ControlBoard getInstance() {
        if (instance == null) {
            instance = new ControlBoard();
        }
        return instance;
    }

    private final DriverControllerIO driverController;
    //private final OperatorControllerIO operatorController;

    private ControlBoard() {
        driverController = DriverController.getInstance();
        //operatorController = OperatorController.getInstance();
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
    public void rumble(boolean intensity) {
        driverController.rumble(intensity);
    }
}
