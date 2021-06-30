package frc.lib.math;

public class PIDGains {
    public final double kP;
    public final double kI;
    public final double kD;
    public final double kFF;
    public final double kMaxForward;
    public final double kMaxReverse;
    
    /**
     * Basic PIDGains. Can be used with Lazy controllers.
     * @param kP
     * @param kI
     * @param kD
     * @param kFF
     */
    public PIDGains(double kP, double kI, double kD, double kFF) {
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
        this.kFF = kFF;
        kMaxForward = 1;
        kMaxReverse = -1;
    }
    
    /**
     * Basic PIDGains with Peak Reverse And Forward Output. Can be used with Lazy Controllers.
     * @param kP
     * @param kI
     * @param kD
     * @param kFF
     * @param kMinOutput
     * @param kMaxOutput
     */
    public PIDGains(double kP, double kI, double kD, double kFF, double kMaxForward, double kMaxReverse) {
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
        this.kFF = kFF;
        this.kMaxForward = kMaxForward;
        this.kMaxReverse = kMaxReverse;
    }
}