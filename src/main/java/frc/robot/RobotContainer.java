package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.States.ShooterStates;
import frc.robot.autos.DriveBack;
import frc.robot.autos.ShootAndDriveBack;
import frc.robot.autos.TrenchRun;
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
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;


public class RobotContainer {
    private final Joystick driver = new Joystick(0);
    private final Joystick operator = new Joystick(1);

    /* Driver Controls */
    private final int forwardAxis = XboxController.Axis.kRightTrigger.value;
    private final int reverseAxis = XboxController.Axis.kLeftTrigger.value;
    private final int rotationAxis = XboxController.Axis.kLeftX.value;
    private final int quickTurnButton = XboxController.Button.kX.value;

    /* Driver Buttons */
    private final JoystickButton intakeButton = new JoystickButton(driver, XboxController.Button.kBumperRight.value);
    private final JoystickButton outakeButton = new JoystickButton(driver, XboxController.Button.kY.value);
    private final POVButton intakeExtendButton = new POVButton(driver, 180);
    private final POVButton intakeRetractButton = new POVButton(driver, 0);
    private final JoystickButton zeroGyroButton = new JoystickButton(driver, XboxController.Button.kStart.value);

    /* Operator Buttons */
    private final JoystickButton shootButton = new JoystickButton(operator, XboxController.Button.kA.value);
    private final JoystickButton shooterActivateButton = new JoystickButton(operator, XboxController.Button.kBumperRight.value);
    private final JoystickButton shooterDeactivateButton = new JoystickButton(operator, XboxController.Button.kBumperLeft.value);

    /* Subsystems */
    private final Vision m_Vision = new Vision();
    private final DriveTrain m_robotDrive = new DriveTrain(m_Vision);
    private final Intake m_Intake = new Intake();
    private final Indexer m_Indexer = new Indexer();
    private final Kicker m_Kicker = new Kicker();
    private final Shooter m_Shooter = new Shooter(m_Vision);

    SendableChooser<Command> autoChooser = new SendableChooser<>();

    public RobotContainer() {
        /* Teleop Drive */
        m_robotDrive.setDefaultCommand(
            new teleopDrive(
                m_robotDrive, 
                () -> (driver.getRawAxis(forwardAxis) - driver.getRawAxis(reverseAxis)), 
                () -> driver.getRawAxis(rotationAxis), 
                () -> driver.getRawButton(quickTurnButton)
            )
        );

        autoChooser.setDefaultOption("Do Nothing", new WaitCommand(1.0));
        autoChooser.addOption("Drive Backwards", new DriveBack(m_robotDrive, m_Kicker, m_Indexer, m_Intake));
        autoChooser.addOption("Shoot and Drive Backwards", new ShootAndDriveBack(m_robotDrive, m_Kicker, m_Indexer, m_Intake));
        autoChooser.addOption("Trench Run", new TrenchRun(m_robotDrive, m_Kicker, m_Indexer, m_Intake));

        SmartDashboard.putData(autoChooser);

        configureButtonBindings();
    }

    private void configureButtonBindings() {
        /* Intake */
        intakeButton.whileHeld(new IntakeControl(m_Intake, 1.0));
        outakeButton.whileHeld(new IntakeControl(m_Intake, -1.0));
        intakeExtendButton.whenPressed(new IntakePistonControl(m_Intake, true));
        intakeRetractButton.whenPressed(new IntakePistonControl(m_Intake, false));

        /* Shooting */
        shootButton.whileHeld(
            new ParallelCommandGroup(
                new KickerControl(m_Kicker, 1.0), 
                new IndexerControl(m_Indexer, 1.0),
                new IntakeControl(m_Intake, 1.0)            
            )
        );
        shooterActivateButton.whenPressed(new InstantCommand(() -> activate_Shooter()));
        shooterDeactivateButton.whenPressed(new InstantCommand(() -> deactivate_Shooter()));

        /* DriveTrain */
        zeroGyroButton.whenPressed(new InstantCommand(() -> zeroGyro()));

    }

    /* Shooter States */
    private void activate_Shooter() {
        States.shooterState = ShooterStates.preShoot;
    }

    private void deactivate_Shooter() {
        States.shooterState = ShooterStates.disabled;
    }

    private void zeroGyro() {
        m_robotDrive.zeroGyro();
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.*
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }
}
