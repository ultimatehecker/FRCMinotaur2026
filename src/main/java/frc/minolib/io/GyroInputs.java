package frc.minolib.io;

import org.littletonrobotics.junction.AutoLog;

import com.ctre.phoenix6.StatusCode;

import edu.wpi.first.math.geometry.Rotation3d;

@AutoLog
public class GyroInputs {
    public boolean isGyroConnected = false;
    public StatusCode status = StatusCode.OK;
    public int faultField = 0;
    public int stickyFaultField = 0;
    public double roll = 0.0;
    public double pitch = 0.0;
    public double yaw = 0.0;
    public double latencyCompensatedYaw = 0.0;
    public double rollRate = 0.0;
    public double pitchRate = 0.0;
    public double yawRate = 0.0;
    public Rotation3d rotation3D = new Rotation3d();
}
