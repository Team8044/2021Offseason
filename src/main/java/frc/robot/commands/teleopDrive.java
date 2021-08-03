package frc.robot.commands;

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
        throttleDouble = (Math.abs(throttle.getAsDouble()) < 0.1) ? 0 : throttle.getAsDouble();
        rotationDouble = (Math.abs(rotation.getAsDouble()) < 0.1) ? 0 : rotation.getAsDouble();

        // if (!quickTurn.getAsBoolean()){
            //     m_driveTrain.curvDrive(throttleDouble, rotationDouble / 1.5, quickTurn.getAsBoolean());
            // }
            // else{
            //     m_driveTrain.curvDrive(throttleDouble, rotationDouble, quickTurn.getAsBoolean());
            // }
        if (throttleDouble < 0){
            throttleDouble = -Math.pow(throttleDouble, 2);
        }
        else {
            throttleDouble = Math.pow(throttleDouble, 2);
        }

        if (rotationDouble < 0){
            rotationDouble = -Math.pow(rotationDouble, 2);
        }
        else {
            rotationDouble = Math.pow(rotationDouble, 2);
        }

        if (!Constants.Shooter.autoAim || States.shooterState != ShooterStates.preShoot){
            m_driveTrain.curvDrive(throttleDouble, rotationDouble, quickTurn.getAsBoolean());
        }
    }
}
