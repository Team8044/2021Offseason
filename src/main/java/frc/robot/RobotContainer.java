package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.States.ShooterStates;
import frc.robot.autos.ExampleAuto;
import frc.robot.commands.IndexerControl;
import frc.robot.commands.IntakeControl;
import frc.robot.commands.IntakePistonControl;
import frc.robot.commands.KickerControl;
import frc.robot.commands.teleopDrive;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Kicker;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Vision;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;


public class RobotContainer {
    private final Joystick driver = new Joystick(0);
    private final Joystick operator = new Joystick(1);

    /* Driver Controls */
    private final int forwardAxis = XboxController.Axis.kLeftY.value;
    // private final int reverseAxis = XboxController.Axis.kLeftTrigger.value;
    private final int rotationAxis = XboxController.Axis.kRightX.value;

    /* Driver Buttons */
    private final JoystickButton intakeButton = new JoystickButton(driver, XboxController.Button.kBumperRight.value);
    private final JoystickButton outakeButton = new JoystickButton(driver, XboxController.Button.kY.value);
    private final POVButton intakeExtendButton = new POVButton(driver, 180);
    private final POVButton intakeRetractButton = new POVButton(driver, 0);

    private final JoystickButton shootButton = new JoystickButton(operator, XboxController.Button.kA.value);
    private final JoystickButton shooterActivateButton = new JoystickButton(operator, XboxController.Button.kBumperRight.value);
    private final JoystickButton shooterDeactivateButton = new JoystickButton(operator, XboxController.Button.kBumperLeft.value);


    /* Operator Buttons */



    /* Subsystems */
    private final Vision m_Vision = new Vision();
    private final DriveTrain m_robotDrive = new DriveTrain();
    private final Intake m_Intake = new Intake();
    private final Indexer m_Indexer = new Indexer();
    private final Kicker m_Kicker = new Kicker();
    private final Shooter m_Shooter = new Shooter(m_Vision);

    public RobotContainer() {
    /* Teleop Drive */
    m_robotDrive.setDefaultCommand(
        new teleopDrive(
            m_robotDrive, 
            m_Vision, 
            () -> -driver.getRawAxis(forwardAxis), 
            () -> driver.getRawAxis(rotationAxis)
        )
    );

    configureButtonBindings();
    }

    private void configureButtonBindings() {
        /* Intake */
        intakeButton.whileHeld(new IntakeControl(m_Intake, 1.0));
        outakeButton.whileHeld(new IntakeControl(m_Intake, -1.0));
        intakeExtendButton.whenPressed(new IntakePistonControl(m_Intake, true));
        intakeRetractButton.whenPressed(new IntakePistonControl(m_Intake,false));

        /* Shooting */
        shootButton.whileHeld(
            new ParallelCommandGroup(
            new KickerControl(m_Kicker, 1), 
            new IndexerControl(m_Indexer, 1),
            new IntakeControl(m_Intake, 1)
            )
        );
        shooterActivateButton.whenPressed(new InstantCommand(() -> activate_Shooter()));
        shooterDeactivateButton.whenPressed(new InstantCommand(() -> deactivate_Shooter()));

    }

    /* Shooter States */
    private void activate_Shooter() {
        States.shooterState = ShooterStates.preShoot;
    }

    private void deactivate_Shooter() {
        States.shooterState = ShooterStates.disabled;
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.*
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        return new ExampleAuto(m_robotDrive).andThen(() -> m_robotDrive.drive(0, 0));
    }
}
