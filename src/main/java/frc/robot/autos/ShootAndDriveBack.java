package frc.robot.autos;

import frc.robot.Constants;
import frc.robot.States;
import frc.robot.States.ShooterStates;
import frc.robot.commands.IndexerControl;
import frc.robot.commands.IntakeControl;
import frc.robot.commands.IntakePistonControl;
import frc.robot.commands.KickerControl;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Kicker;

import java.util.List;

import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

// Start directly in front of the goal

public class ShootAndDriveBack extends SequentialCommandGroup {

    public ShootAndDriveBack(DriveTrain m_robotDrive, Kicker m_Kicker, Indexer m_Indexer, Intake m_Intake) {
        addRequirements(m_robotDrive);

        RamseteCommand driveBackwards =
            new RamseteCommand(
                TrajectoryGenerator.generateTrajectory(
                    new Pose2d(0, 0, new Rotation2d(0)), 
                    List.of(                              
                        new Translation2d(-2,0)
                    ), 
                    new Pose2d(-2, 0, new Rotation2d(0)), 
                    Constants.AutoConstants.trajConfig.setReversed(true)
                ),
                m_robotDrive::getPose,
                new RamseteController(Constants.AutoConstants.kRamseteB, Constants.AutoConstants.kRamseteZeta),
                Constants.Drive.driveKinematics,
                (leftspeed, rightspeed) -> m_robotDrive.setWheelState(leftspeed, rightspeed),
                m_robotDrive
            );

        addCommands(
            new InstantCommand(() -> m_robotDrive.zeroGyro()),
            new InstantCommand(() -> m_robotDrive.resetOdometry(new Pose2d(0, 0, new Rotation2d(0)))),
            new WaitCommand(0.1),
            new IntakePistonControl(m_Intake, true),

            new InstantCommand(() -> States.shooterState = ShooterStates.preShoot),
            new WaitCommand(1.0), //Wait for 1 second to auto aim and spin up before shooting
            new ParallelDeadlineGroup(
                new WaitCommand(5.0), //Shoot for 5 seconds
                new KickerControl(m_Kicker, 1.0),
                new IndexerControl(m_Indexer, 1.0),
                new IntakeControl(m_Intake, 1.0)
            ),
            new InstantCommand(() -> States.shooterState = ShooterStates.disabled),

            driveBackwards,
            new InstantCommand(() -> m_robotDrive.arcadeDrive(0, 0, false))
        );
    }
}
