package frc.robot.autos;

import frc.robot.Constants;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Kicker;

import java.util.List;

import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

// Start directly in front of the goal

public class DriveBack extends SequentialCommandGroup {

    public DriveBack(DriveTrain m_robotDrive, Kicker m_Kicker, Indexer m_Indexer, Intake m_Intake) {
        addRequirements(m_robotDrive);

        Trajectory driveBackwards = TrajectoryGenerator.generateTrajectory(
            List.of( 
                new Pose2d(0, 0, new Rotation2d(0)), 
                new Pose2d(-2, 0, new Rotation2d(0))
            ), 
            new TrajectoryConfig(3.0, 3.0).setKinematics(Constants.Drive.driveKinematics).setReversed(true)
        );

        addCommands(
            new InstantCommand(() -> m_robotDrive.zeroGyro()),
            new InstantCommand(() -> m_robotDrive.resetOdometry(driveBackwards.getInitialPose())),
            
            /* Drive backwards */
            new RamseteCommand(
                driveBackwards,
                m_robotDrive::getPose,
                new RamseteController(Constants.AutoConstants.kRamseteB, Constants.AutoConstants.kRamseteZeta),
                Constants.Drive.driveKinematics,
                (leftspeed, rightspeed) -> m_robotDrive.setWheelState(leftspeed, rightspeed),
                m_robotDrive
            ),
            new InstantCommand(() -> m_robotDrive.arcadeDrive(0, 0, false))
        );
    }
}
