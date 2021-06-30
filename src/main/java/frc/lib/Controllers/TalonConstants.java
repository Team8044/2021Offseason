package frc.lib.Controllers;

import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;

public class TalonConstants {
    public final int deviceNumber;
    public final SupplyCurrentLimitConfiguration currentLimit;
    public final NeutralMode neutralMode;
    public final InvertType invertType;
    
    /**
     * Constants to be used with LazyTalonFX Util
     * @param deviceNumber
     * @param currentLimit
     * @param neutralMode
     * @param invertType
     */
    public TalonConstants(int deviceNumber, SupplyCurrentLimitConfiguration currentLimit, NeutralMode neutralMode, InvertType invertType) {
        this.deviceNumber = deviceNumber;
        this.currentLimit = currentLimit;
        this.neutralMode = neutralMode;
        this.invertType = invertType;
    }
}