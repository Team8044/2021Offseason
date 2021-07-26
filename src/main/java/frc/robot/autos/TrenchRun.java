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
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.util.Units;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

/* Start o the side of the goal, shoot, trench run, pickup 3 balls, return to goal, shoot */

public class TrenchRun extends SequentialCommandGroup {

    public TrenchRun(DriveTrain m_robotDrive, Kicker m_Kicker, Indexer m_Indexer, Intake m_Intake) {
        addRequirements(m_robotDrive);

        Trajectory driveToTrench = TrajectoryGenerator.generateTrajectory(
            new Pose2d(Units.feetToMeters(42.167), Units.feetToMeters(4.236), Rotation2d.fromDegrees(16)), 
            List.of(                              
                new Translation2d(Units.feetToMeters(35.089), Units.feetToMeters(2.27)),
                new Translation2d(Units.feetToMeters(32.089), Units.feetToMeters(2.27)),
                new Translation2d(Units.feetToMeters(29.089), Units.feetToMeters(2.27))
            ), 
            new Pose2d(Units.feetToMeters(27.334), Units.feetToMeters(2.27), Rotation2d.fromDegrees(0)), 
            Constants.AutoConstants.trajConfig.setReversed(true)
        );

        Trajectory driveToTarget = TrajectoryGenerator.generateTrajectory(
            new Pose2d(Units.feetToMeters(27.334), Units.feetToMeters(2.27), Rotation2d.fromDegrees(0)), 
            List.of( 
                new Translation2d(Units.feetToMeters(29.089), Units.feetToMeters(2.27)),
                new Translation2d(Units.feetToMeters(32.089), Units.feetToMeters(2.27)),
                new Translation2d(Units.feetToMeters(35.089), Units.feetToMeters(2.27))
            ), 
            new Pose2d(Units.feetToMeters(42.167), Units.feetToMeters(4.236), Rotation2d.fromDegrees(16)), 
            Constants.AutoConstants.trajConfig.setReversed(false)
        );
            

        addCommands(
            new InstantCommand(() -> m_robotDrive.zeroGyro()),
            new InstantCommand(() -> m_robotDrive.resetOdometry(new Pose2d(driveToTrench.getInitialPose().getTranslation(), new Rotation2d()))),
            new IntakePistonControl(m_Intake, true),

            /* Shoot from Starting Line */
            new InstantCommand(() -> States.shooterState = ShooterStates.preShoot),
            new WaitCommand(1.0), //Wait for 1 second to auto aim and spin up before shooting
            new ParallelDeadlineGroup(
                new WaitCommand(2.5), 
                new KickerControl(m_Kicker, 1.0),
                new IndexerControl(m_Indexer, 1.0),
                new IntakeControl(m_Intake, 1.0)
            ),
            new InstantCommand(() -> States.shooterState = ShooterStates.disabled),

            /* Drive through trench and pickup balls */
            new ParallelDeadlineGroup(
                new RamseteCommand(
                    driveToTrench,
                    m_robotDrive::getPose,
                    new RamseteController(Constants.AutoConstants.kRamseteB, Constants.AutoConstants.kRamseteZeta),
                    Constants.Drive.driveKinematics,
                    (leftspeed, rightspeed) -> m_robotDrive.setWheelState(leftspeed, rightspeed),
                    m_robotDrive
                ),
                new IntakeControl(m_Intake, 1.0)
            ),
            new InstantCommand(() -> m_robotDrive.arcadeDrive(0, 0, false)),

            /* Drive to goal and shoot */
            new ParallelDeadlineGroup(
                new RamseteCommand(
                    driveToTarget,
                    m_robotDrive::getPose,
                    new RamseteController(Constants.AutoConstants.kRamseteB, Constants.AutoConstants.kRamseteZeta),
                    Constants.Drive.driveKinematics,
                    (leftspeed, rightspeed) -> m_robotDrive.setWheelState(leftspeed, rightspeed),
                    m_robotDrive
                )
                // new IntakeControl(m_Intake, 1.0)
            ),
            new InstantCommand(() -> m_robotDrive.arcadeDrive(0, 0, false)),

            new InstantCommand(() -> States.shooterState = ShooterStates.preShoot),
            new WaitCommand(1.0), //Wait for 1 second to auto aim and spin up before shooting
            new ParallelDeadlineGroup(
                new WaitCommand(3.0), //Shoot for 5 seconds
                new KickerControl(m_Kicker, 1.0),
                new IndexerControl(m_Indexer, 1.0),
                new IntakeControl(m_Intake, 1.0)
            ),
            new InstantCommand(() -> States.shooterState = ShooterStates.disabled),
            new IntakePistonControl(m_Intake, true)
        );
    }
}
