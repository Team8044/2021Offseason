package frc.lib.Controllers;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import frc.lib.math.PIDGains;

/**
 * Thin WPI Falcon wrapper to make setup easier and reduce CAN bus overhead 
 * by skipping duplicate commands.
 */
public class WPI_LazyTalonFX extends WPI_TalonFX {

    /**
     * Config a Talon FX using talonFxConstants.
     * 
     * @param talonFxConstants
     */
    public WPI_LazyTalonFX(TalonConstants talonConstants) {
        super(talonConstants.deviceNumber);
        super.configFactoryDefault();
        super.configSupplyCurrentLimit(talonConstants.currentLimit);
        super.setNeutralMode(talonConstants.neutralMode);
        super.setInverted(talonConstants.invertType);
        super.configVoltageCompSaturation(12);
        super.enableVoltageCompensation(true);
        super.setSelectedSensorPosition(0);
    }
    
    /**
     * Config PID Gains and Peak Outputs using PIDGains
     * @param pidGains
     */
    public void configPID(PIDGains pidGains){
        super.config_kP(0, pidGains.kP);
        super.config_kI(0, pidGains.kI);
        super.config_kD(0, pidGains.kD);
        super.config_kF(0, pidGains.kFF);
        super.configPeakOutputForward(pidGains.kMaxForward);
        super.configPeakOutputReverse(pidGains.kMaxReverse);
    }
    
}