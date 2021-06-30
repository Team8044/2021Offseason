package frc.lib.Controllers;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSensor;
import com.revrobotics.CANSparkMax;
import com.revrobotics.ControlType;

import frc.lib.math.PIDGains;

/**
 * Thin Spark Max wrapper to make setup easier and reduce CAN bus overhead 
 * by skipping duplicate commands.
 */
public class LazySparkMAX extends CANSparkMax {
    protected double lastSet = Double.NaN;
    protected ControlType lastControlType = null;
    protected CANPIDController m_pidController;
    protected CANEncoder m_encoder;

    /**
     * Config a Spark Max using sparkConstants.
     * 
     * @param sparkConstants
     */
    public LazySparkMAX(SparkConstants sparkConstants) {
        super(sparkConstants.deviceId, sparkConstants.motorType);
        super.restoreFactoryDefaults();
        super.setSmartCurrentLimit(sparkConstants.smartCurrentLimit);
        super.enableVoltageCompensation(12);
        super.setIdleMode(sparkConstants.idleMode);
        super.setInverted(sparkConstants.inverted);
        super.burnFlash();

        m_pidController = super.getPIDController();
        m_encoder = super.getEncoder();

        m_encoder.setPosition(0);
    }

    /**
     * Config a Spark Max follower using sparkConstants, master Spark Max to follow,
     * and if the follower should be inverted or not from the master.
     * 
     * @param sparkConstants
     * @param masterSpark Master Spark Max to follow
     * @param invertedFromMaster Is follower inverted from master.
     */
    public LazySparkMAX(SparkConstants sparkConstants, CANSparkMax masterSpark, boolean invertedFromMaster) {
        super(sparkConstants.deviceId, sparkConstants.motorType);
        super.restoreFactoryDefaults();
        super.setSmartCurrentLimit(sparkConstants.smartCurrentLimit);
        super.enableVoltageCompensation(12);
        super.setIdleMode(sparkConstants.idleMode);
        super.follow(masterSpark, invertedFromMaster);
        super.burnFlash();

        m_pidController = super.getPIDController();
        m_encoder = super.getEncoder();

        m_encoder.setPosition(0);
    }
    
    public void set(ControlType type, double setpoint) {
        m_pidController.setReference(setpoint, type);
    }
    
    /**
     * Config PID Gains and Peak Outputs using PIDGains
     * @param pidGains
     */
    public void configPID(PIDGains pidGains){
        m_pidController.setP(pidGains.kP);
        m_pidController.setI(pidGains.kI);
        m_pidController.setD(pidGains.kD);
        m_pidController.setFF(pidGains.kFF);
        // m_pidController.setOutputRange(pidGains.kMaxReverse, pidGains.kMaxForward);
    }

    /**
     * Returns PIDGains and configured Peak Outputs from internal PIDController
     * @return
     */
    public PIDGains getPIDGains(){
        PIDGains pidGains = new PIDGains(
            m_pidController.getP(), 
            m_pidController.getI(), 
            m_pidController.getD(), 
            m_pidController.getFF(), 
            m_pidController.getOutputMax(), 
            m_pidController.getOutputMin()
        );
        return pidGains;
    }

    /**
     * Set the internal pid controllers feedback device.
     * @param sensor
     */
    public void setFeedbackDevice(CANSensor sensor) {
        m_pidController.setFeedbackDevice(sensor);
    }

    public double getPosition(){
        return m_encoder.getPosition();
    }

    public void setPosition(double position){
        m_encoder.setPosition(position);
    }

    public double getVelocity(){
        return m_encoder.getVelocity();
    }

    /**
     * Set conversion factor of encoder, so that position is in meters and velocity is in MPS.
     * @param circumference Circumference of wheel in meters
     * @param gearRatio Reduction to wheel. For example "15" if on a 15:1 reduction.
     */
    public void setConversionFactorMeters(double circumference, double gearRatio){
        m_encoder.setPositionConversionFactor(circumference / gearRatio);
        m_encoder.setVelocityConversionFactor((circumference / gearRatio) / 60.0);
    }

    /**
     * Set conversion factor of encoder, so that position is in rotations and velocity is in RPM
     * @param gearRatio Reduction to mech. For example "15" if on a 15:1 reduction.
     */
    public void setConversionFactorRotations(double gearRatio){
        m_encoder.setPositionConversionFactor(1 / gearRatio);
        m_encoder.setVelocityConversionFactor(1 / gearRatio);
    }

    /**
     * Set conversion factor of encoder, so that position is in degrees and velocity is in degrees/min
     * @param gearRatio Reduction to mech. For example "15" if on a 15:1 reduction.
     */
    public void setConversionFactorDegrees(double gearRatio){
        m_encoder.setPositionConversionFactor(360 / gearRatio);
        m_encoder.setVelocityConversionFactor(360 / gearRatio);
    }



    
}