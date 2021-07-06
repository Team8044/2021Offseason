package frc.lib.Controllers;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import frc.lib.math.PIDGains;

/**
 * Thin Victor SPX wrapper to make setup easier.
 */
public class LazyVictorSPX extends VictorSPX {

    /**
     * Config a Victor SPX using talonConstants
     * 
     * @param talonConstants
     */
    public LazyVictorSPX(TalonConstants talonConstants) {
        super(talonConstants.deviceNumber);
        super.configFactoryDefault();
        // super.configSupplyCurrentLimit(talonConstants.currentLimit);
        super.setNeutralMode(talonConstants.neutralMode);
        super.setInverted(talonConstants.invertType);
        super.configVoltageCompSaturation(12);
        super.enableVoltageCompensation(true);
        super.setSelectedSensorPosition(0);
    }

    /**
     * Config a Victor SPX slave using talonFxConstants and master Talon.
     * 
     * @param talonConstants
     * @param masterTalon Talon to Follow
     */
    public LazyVictorSPX(TalonConstants talonConstants, TalonSRX masterTalon) {
        super(talonConstants.deviceNumber);
        super.configFactoryDefault();
        // super.configSupplyCurrentLimit(talonConstants.currentLimit);
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