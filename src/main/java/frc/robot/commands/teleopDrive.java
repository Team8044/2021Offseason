package frc.robot.commands;

import frc.lib.util.Limelight;
import frc.robot.States;
import frc.robot.States.ShooterStates;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Vision;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class teleopDrive extends CommandBase {
    private final DriveTrain m_subsystem;
    private final Limelight m_Limelight;

    DoubleSupplier throttle;
    DoubleSupplier rotation;

    private final TrapezoidProfile.Constraints m_constraints;
    private final ProfiledPIDController m_controller;

    public teleopDrive(DriveTrain subsystem, Vision m_Vision, DoubleSupplier throttle, DoubleSupplier rotation) {
        m_subsystem = subsystem;
        addRequirements(m_subsystem);
        m_Limelight = m_Vision.limelight;

        this.throttle = throttle;
        this.rotation = rotation;

        m_constraints = new TrapezoidProfile.Constraints(1.75, 1.75);
        m_controller = new ProfiledPIDController(0.08, 0.05, 0.005, m_constraints);
        m_controller.setGoal(0);
    }

    @Override
    public void execute() {
        if (States.shooterState == ShooterStates.preShoot && m_Limelight.hasTarget()){
            m_subsystem.drive(throttle.getAsDouble(), m_controller.calculate(m_Limelight.getTx().getDegrees()), true);
        } else{
            m_subsystem.drive(throttle.getAsDouble(), rotation.getAsDouble(), true);
        }
    }
}
