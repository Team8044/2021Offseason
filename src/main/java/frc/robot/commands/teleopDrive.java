package frc.robot.commands;

import frc.lib.util.Limelight;
import frc.robot.States;
import frc.robot.States.ShooterStates;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Vision;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class teleopDrive extends CommandBase {
    private final DriveTrain m_driveTrain;
    private final Limelight m_Limelight;

    DoubleSupplier throttle;
    DoubleSupplier rotation;
    BooleanSupplier quickTurn;

    double throttleDouble;
    double rotationDouble;

    private final TrapezoidProfile.Constraints m_constraints;
    private final ProfiledPIDController m_controller;

    public teleopDrive(DriveTrain m_driveTrain, Vision m_Vision, DoubleSupplier throttle, DoubleSupplier rotation, BooleanSupplier quickTurn) {
        this.m_driveTrain = m_driveTrain;
        addRequirements(m_driveTrain);
        m_Limelight = m_Vision.limelight;

        this.throttle = throttle;
        this.rotation = rotation;
        this.quickTurn = quickTurn;

        m_constraints = new TrapezoidProfile.Constraints(1.75, 1.75);
        m_controller = new ProfiledPIDController(0.08, 0.05, 0.005, m_constraints);
        m_controller.setGoal(0);
    }

    @Override
    public void execute() {
        /* Deadbands */
        throttleDouble = (Math.abs(throttle.getAsDouble()) < 0.1) ? 0 : throttle.getAsDouble();
        rotationDouble = (Math.abs(rotation.getAsDouble()) < 0.1) ? 0 : rotation.getAsDouble();

        if (States.shooterState == ShooterStates.preShoot && m_Limelight.hasTarget()){
            // m_driveTrain.arcadeDrive(throttle.getAsDouble(), m_controller.calculate(m_Limelight.getTx().getDegrees()), true);
            m_driveTrain.autoAim(throttleDouble);
        } else{
            m_driveTrain.curvDrive(throttleDouble, rotationDouble, quickTurn.getAsBoolean());
        }
    }
}
