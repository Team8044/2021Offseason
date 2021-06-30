package frc.robot;

public class States {

    /**
     * Shooter States:
     * </p> disabled: Shooter Disabled and Shooter Tilt at 0
     * </p> preShoot: Shooter SpinUp, Shooter Tilt to angle, drivetrain target lock
     */
    public static enum ShooterStates {
        notCalibrated, disabled, preShoot
    }

    public static ShooterStates shooterState = ShooterStates.notCalibrated;
}
