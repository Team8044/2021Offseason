package frc.robot.commands;

import frc.lib.math.Boundaries;
import frc.robot.Constants;
import frc.robot.States;
import frc.robot.States.ShooterStates;
import frc.robot.subsystems.DriveTrain;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class teleopDrive extends CommandBase {
    private final DriveTrain m_driveTrain;
    
    DoubleSupplier throttle;
    DoubleSupplier rotation;
    BooleanSupplier quickTurn;

    double throttleDouble;
    double rotationDouble;

    public teleopDrive(DriveTrain m_driveTrain, DoubleSupplier throttle, DoubleSupplier rotation, BooleanSupplier quickTurn) {
        this.m_driveTrain = m_driveTrain;
        addRequirements(m_driveTrain);

        this.throttle = throttle;
        this.rotation = rotation;
        this.quickTurn = quickTurn;
    }

    @Override
    public void execute() {
        /* Deadbands */
        throttleDouble = (Math.abs(throttle.getAsDouble()) < Constants.Drive.controllerDeadband) ? 0 : throttle.getAsDouble();
        rotationDouble = (Math.abs(rotation.getAsDouble()) < Constants.Drive.controllerDeadband) ? 0 : rotation.getAsDouble();

        /* Input Curving */
        throttleDouble = Boundaries.curveInput(throttleDouble, Constants.Drive.throttleCurve);
        rotationDouble = Boundaries.curveInput(rotationDouble, Constants.Drive.rotationCurve);

        /* Limiting Max Output */
        throttleDouble = (Math.abs(throttleDouble) > Constants.Drive.maxThrottle) ? (Constants.Drive.maxThrottle * Math.signum(throttleDouble)) : throttleDouble;
        rotationDouble = (Math.abs(rotationDouble) > Constants.Drive.maxRotation) ? (Constants.Drive.maxRotation * Math.signum(rotationDouble)) : rotationDouble;

        if (!Constants.Shooter.autoAim || States.shooterState != ShooterStates.preShoot){
            m_driveTrain.curvDrive(throttleDouble, rotationDouble, quickTurn.getAsBoolean());
        }
    }
}
