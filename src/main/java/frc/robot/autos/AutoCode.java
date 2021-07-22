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
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

// Start directly in front of the goal

public class AutoCode extends SequentialCommandGroup {

    /// An example trajectory.  All units in meters.
  private Trajectory exampleTrajectory = 
  TrajectoryGenerator.generateTrajectory(
      new Pose2d(0, 0, new Rotation2d(0)), // Start at the origin facing the +X direction
      List.of(                              // Pass through these two interior waypoints, making an 's' curve path
        new Translation2d(-2,0)
      ), 
      new Pose2d(-2, 0, new Rotation2d(0)), // End 3 meters straight ahead of where we started, facing forward
    Constants.AutoConstants.trajConfig.setReversed(true));

    public AutoCode(DriveTrain robotDrive, Kicker Kicker, Indexer Indexer, Intake Intake) {
        addRequirements(robotDrive);

        RamseteCommand ramseteCommand =
        new RamseteCommand(
            exampleTrajectory,
            robotDrive::getPose,
            new RamseteController(Constants.AutoConstants.kRamseteB, Constants.AutoConstants.kRamseteZeta),
            Constants.Drive.driveKinematics,
            (leftspeed, rightspeed) -> robotDrive.setWheelState(leftspeed, rightspeed),
            robotDrive);

        addCommands(
            new InstantCommand(() -> robotDrive.zeroGyro()),
            new InstantCommand(() -> robotDrive.resetOdometry(exampleTrajectory.getInitialPose())),
            new WaitCommand(0.1),
            new IntakePistonControl(Intake, true),

            new InstantCommand(() -> States.shooterState = ShooterStates.preShoot),
            new WaitCommand(1.0), //Wait for 1 second to auto aim and spin up before shooting
            new ParallelDeadlineGroup(
                new WaitCommand(5.0), //Shoot for 5 seconds
                new KickerControl(Kicker, 1.0),
                new IndexerControl(Indexer, 1.0),
                new IntakeControl(Intake, 1.0)
            ),
            new InstantCommand(() -> States.shooterState = ShooterStates.disabled),
            ramseteCommand
            
            
        );
    }
}
