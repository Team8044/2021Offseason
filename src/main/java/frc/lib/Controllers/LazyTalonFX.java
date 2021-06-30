package frc.lib.Controllers;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import frc.lib.math.PIDGains;

/**
 * Thin Falcon wrapper to make setup easier and reduce CAN bus overhead 
 * by skipping duplicate commands.
 */
public class LazyTalonFX extends TalonFX {
    protected double lastSet = Double.NaN;
    protected ControlMode lastControlMode = null;

    /**
     * Config a Talon FX using talonFxConstants.
     * 
     * @param talonConstants
     */
    public LazyTalonFX(TalonConstants talonConstants) {
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
     * Config a Talon FX slave using talonFxConstants and master.
     * 
     * @param talonConstants
     * @param masterTalon Talon FX to follow
     */
    public LazyTalonFX(TalonConstants talonConstants, TalonFX masterTalon) {
        super(talonConstants.deviceNumber);
        super.configFactoryDefault();
        super.configSupplyCurrentLimit(talonConstants.currentLimit);
        super.setNeutralMode(talonConstants.neutralMode);
        super.setInverted(talonConstants.invertType);
        super.configVoltageCompSaturation(12);
        super.enableVoltageCompensation(true);
        super.setSelectedSensorPosition(0);
        super.follow(masterTalon);
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
        super.configNominalOutputForward(0);
        super.configNominalOutputReverse(0);
        super.configPeakOutputForward(pidGains.kMaxForward);
        super.configPeakOutputReverse(pidGains.kMaxReverse);
    }
    
}