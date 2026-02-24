package frc.minolib.hardware;

/**
 * Represents a CAN device identifier with bus name and device number. Used to uniquely identify and
 * configure CAN-based devices on the robot.
 */

public class MinoCANDevice {
    /** Default CAN bus name for the RoboRIO */
    public static final MinoCANBus kRIOCANbusName = new MinoCANBus("rio");
  
    /** The device number on the CAN bus */
    public int deviceNumber;
  
    /** The name of the CAN bus this device is on */
    public final MinoCANBus CANbusName;
  
    /**
     * Creates a new CAN device ID with specified device number and bus name.
     *
     * @param deviceNumber The device's CAN ID number
     * @param CANbusName The name of the CAN bus
     */
    public MinoCANDevice(final int deviceNumber, final MinoCANBus CANbusName) {
        this.deviceNumber = deviceNumber;
        this.CANbusName = CANbusName;
    }
  
    /**
     * Creates a new CAN device ID on the default RIO CAN bus.
     *
     * @param deviceNumber The device's CAN ID number
     */
    public MinoCANDevice(final int deviceNumber) {
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