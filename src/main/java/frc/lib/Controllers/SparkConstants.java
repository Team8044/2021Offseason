package frc.lib.Controllers;

import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

public class SparkConstants {
    public final int deviceId;
    public final MotorType motorType;
    public final int smartCurrentLimit;
    public final IdleMode idleMode;
    public final boolean inverted;

    public SparkConstants(int deviceId, MotorType motorType, int smartCurrentLimit, IdleMode idleMode, boolean inverted) {
        this.deviceId = deviceId;
        this.motorType = motorType;
        this.smartCurrentLimit = smartCurrentLimit;
        this.idleMode = idleMode;
        this.inverted = inverted;
    }
}