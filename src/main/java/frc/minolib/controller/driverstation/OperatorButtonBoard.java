package frc.minolib.controller.driverstation;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class OperatorButtonBoard {
    private final GenericHID operatorHID;

    public OperatorButtonBoard(int port) {
        operatorHID = new GenericHID(port);
    }

    public boolean isConnected() {
        return operatorHID.isConnected();
    }

    public boolean getSPSTSwitch(int index) {
        if (index < 7 || index > 11) {
            throw new RuntimeException("Invalid driver override index " + Integer.toString(index) + ". Must be 7-11.");
        }

        return operatorHID.getRawButton(index + 1);
    }

    public boolean getMomentarySwitch(int index) {
        if (index < 0 || index > 5) {
            throw new RuntimeException("Invalid operator override index " + Integer.toString(index) + ". Must be 0-5.");
        }

        return operatorHID.getRawButton(index + 8);
    }

    public Trigger a() {
        return new Trigger(() -> getSPSTSwitch(11));
    }

    public Trigger b() {
        return new Trigger(() -> getSPSTSwitch(10));
    }

    public Trigger c() {
        return new Trigger(() -> getSPSTSwitch(9));
    }

    public Trigger d() {
        return new Trigger(() -> getSPSTSwitch(8));
    }

    public Trigger e() {
        return new Trigger(() -> getSPSTSwitch(7));
    }

    public Trigger z() {
        return new Trigger(() -> getMomentarySwitch(5));
    }

    public Trigger y() {
        return new Trigger(() -> getMomentarySwitch(4));
    }

    public Trigger w() {
        return new Trigger(() -> getMomentarySwitch(3));
    }

    public Trigger v() {
        return new Trigger(() -> getMomentarySwitch(2));
    }

    public Trigger u() {
        return new Trigger(() -> getMomentarySwitch(1));
    }

    public Trigger t() {
        return new Trigger(() -> getMomentarySwitch(0));
    }
}