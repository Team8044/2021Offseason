package frc.robot.autos;

import frc.robot.Constants;
import frc.robot.subsystems.DriveTrain;

import java.util.List;

import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;


public class ExampleAuto extends SequentialCommandGroup {

    // An example trajectory.  All units in meters.
    private Trajectory exampleTrajectory = 
        TrajectoryGenerator.generateTrajectory(
            new Pose2d(0, 0, new Rotation2d(0)),
            List.of(                              
                new Translation2d(2, 2), 
                new Translation2d(3, -2)
            ), 
            new Pose2d(5, 2, new Rotation2d(0)),
        Constants.AutoConstants.trajConfig);

    public ExampleAuto(DriveTrain m_robotDrive) {
        addRequirements(m_robotDrive);

        RamseteCommand ramseteCommand =
            new RamseteCommand(
                exampleTrajectory,
                m_robotDrive::getPose,
                new RamseteController(Constants.AutoConstants.kRamseteB, Constants.AutoConstants.kRamseteZeta),
                Constants.Drive.driveKinematics,
                (leftspeed, rightspeed) -> m_robotDrive.setWheelState(leftspeed, rightspeed),
                m_robotDrive);

        addCommands(
        new InstantCommand(() -> m_robotDrive.zeroGyro()),
        new InstantCommand(() -> m_robotDrive.resetOdometry(exampleTrajectory.getInitialPose())),
        ramseteCommand
        );
    }

}
