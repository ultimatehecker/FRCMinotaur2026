package frc.minolib.hardware;

/**
 * Represents a CAN device identifier with bus name and device number. Used to uniquely identify and
 * configure CAN-based devices on the robot.
 */

public class CANDeviceID {
    /** Default CAN bus name for the RoboRIO */
    public static final String kRIOCANbusName = "rio";
  
    /** The device number on the CAN bus */
    public int deviceNumber;
  
    /** The name of the CAN bus this device is on */
    public final String CANbusName;
  
    /**
     * Creates a new CAN device ID with specified device number and bus name.
     *
     * @param deviceNumber The device's CAN ID number
     * @param CANbusName The name of the CAN bus
     */
    public CANDeviceID(final int deviceNumber, final String CANbusName) {
        this.deviceNumber = deviceNumber;
        this.CANbusName = CANbusName;
    }
  
    /**
     * Creates a new CAN device ID on the default RIO CAN bus.
     *
     * @param deviceNumber The device's CAN ID number
     */
    public CANDeviceID(final int deviceNumber) {
        this(deviceNumber, kRIOCANbusName);
    }

    public void newDeviceID(final int deviceNumber) {
        this.deviceNumber = deviceNumber;
    }
  
    /**
     * @return String representation of the CAN device ID in format [busName deviceNumber]
     */
    @Override
    public String toString() {
        return "[" + CANbusName + " " + deviceNumber + "]";
    }
  }