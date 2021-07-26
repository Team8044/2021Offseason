package frc.robot;

public class States {

    /**
     * Shooter States:
     * </p> disabled: Auto Aim Disabled, Shooter Disabled, and Shooter Tilt at 0
     * </p> preShoot: Drivetrain Auto Aim, Shooter SpinUp, Shooter Tilt to angle
     */
    public static enum ShooterStates {
        disabled, preShoot
    }

    public static ShooterStates shooterState = ShooterStates.disabled;
}
