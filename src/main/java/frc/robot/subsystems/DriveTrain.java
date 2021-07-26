package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.Controllers.WPI_LazyTalonFX;
import frc.lib.math.Boundaries;
import frc.lib.math.Conversions;
import frc.lib.util.Limelight;
import frc.robot.Constants;
import frc.robot.States;

public class DriveTrain extends SubsystemBase {
    private WPI_LazyTalonFX leftMaster;
    private WPI_LazyTalonFX leftSlave;
    private WPI_LazyTalonFX rightMaster;
    private WPI_LazyTalonFX rightSlave;

    private DifferentialDrive m_robotDrive;
    private DifferentialDriveOdometry m_odometry;
    private SimpleMotorFeedforward driveFF;
    
    private final ProfiledPIDController m_controller;
    private final Limelight m_Limelight;

    private int currentNeutral = 0;
    private DriverStation ds;

    private AHRS gyro;

    // private double previousP = 0;

    public DriveTrain(Vision m_Vision) {
        leftMaster = new WPI_LazyTalonFX(Constants.Drive.leftMaster);
        leftSlave = new WPI_LazyTalonFX(Constants.Drive.leftSlave);
        rightMaster = new WPI_LazyTalonFX(Constants.Drive.rightMaster);
        rightSlave = new WPI_LazyTalonFX(Constants.Drive.rightSlave);

        leftMaster.configPID(Constants.Drive.drivePID);
        rightMaster.configPID(Constants.Drive.drivePID);

        leftSlave.follow(leftMaster);
        rightSlave.follow(rightMaster);

        gyro = new AHRS();
        zeroGyro();

        m_robotDrive = new DifferentialDrive(leftMaster, rightMaster);
        m_robotDrive.setRightSideInverted(false);
        m_robotDrive.setSafetyEnabled(false);

        m_odometry = new DifferentialDriveOdometry(getYaw());
        driveFF = new SimpleMotorFeedforward(Constants.Drive.drivekS / 12, Constants.Drive.drivekV / 12, Constants.Drive.drivekA / 12);
        
        m_controller = new ProfiledPIDController(Constants.Drive.autoAimPID.kP, Constants.Drive.autoAimPID.kI, Constants.Drive.autoAimPID.kD, Constants.Drive.autoAimConstraints);
        m_controller.setGoal(0);
        m_Limelight = m_Vision.limelight;
        ds = DriverStation.getInstance();

        
        // SmartDashboard.putNumber("Drive p", 0);
        // SmartDashboard.putNumber("Drive mps", 0);
    }

    /* For standard Teleop Drive */
    public void arcadeDrive(double speed, double rotation, Boolean squaredInput){
        m_robotDrive.arcadeDrive(speed, rotation, squaredInput);
    }
    
    public void curvDrive(double speed, double rotation, Boolean quickTurn){
        m_robotDrive.curvatureDrive(speed, rotation, quickTurn);
    }

    /* Used for RamseteController for Following Auto Paths */
    public void setWheelState(double leftSpeed, double rightSpeed){
        double leftDemand = Conversions.MPSToFalcon(leftSpeed, Constants.Drive.wheelCircumference, Constants.Drive.gearRatio);
        double rightDemand = Conversions.MPSToFalcon(rightSpeed, Constants.Drive.wheelCircumference, Constants.Drive.gearRatio);

        leftMaster.set(ControlMode.Velocity, leftDemand, DemandType.ArbitraryFeedForward, driveFF.calculate(leftDemand));
        rightMaster.set(ControlMode.Velocity, rightDemand, DemandType.ArbitraryFeedForward, driveFF.calculate(rightDemand));
    }

    public Rotation2d getYaw() {
        double yaw = Boundaries.to360Boundaries(gyro.getAngle());
        return Constants.Drive.invertGyro ? Rotation2d.fromDegrees(360 - yaw) : Rotation2d.fromDegrees(yaw);
    }

    public void zeroGyro(){
        gyro.zeroYaw();
    }

    public void resetOdometry(Pose2d pose) {
        leftMaster.setSelectedSensorPosition(0);
        rightMaster.setSelectedSensorPosition(0);
        m_odometry.resetPosition(pose, getYaw());
    }

    public void setNeutral(NeutralMode neutral){
        leftMaster.setNeutralMode(neutral);
        leftSlave.setNeutralMode(neutral);
        rightMaster.setNeutralMode(neutral);
        rightSlave.setNeutralMode(neutral);
      }

    public Pose2d getPose() {
        return m_odometry.getPoseMeters();
    }

    public double getLeftPos(){
        return Conversions.falconToMeters(
            leftMaster.getSelectedSensorPosition(), 
            Constants.Drive.wheelCircumference, 
            Constants.Drive.gearRatio
        );
    }

    public double getRightPos(){
        return Conversions.falconToMeters(
            rightMaster.getSelectedSensorPosition(), 
            Constants.Drive.wheelCircumference, 
            Constants.Drive.gearRatio
        );
    }

    @Override
    public void periodic() {
        m_odometry.update(getYaw(), getLeftPos(), getRightPos());
        
        switch(States.shooterState){
            case disabled:
                break;
                
            case preShoot:
                m_robotDrive.arcadeDrive(0.0, m_controller.calculate(m_Limelight.getTx().getDegrees()), false);
                break;
        }

        /* Automatical set to coast if disabled */
        if (ds.isEnabled() && currentNeutral == 1){
            setNeutral(NeutralMode.Brake);
            currentNeutral = 0;
          }
          
          if (ds.isDisabled() && currentNeutral == 0){
            setNeutral(NeutralMode.Coast);
            currentNeutral = 1;
          }

        // double mps = SmartDashboard.getNumber("Drive mps", 0);
        // setWheelState(mps, mps);

        // if (previousP != SmartDashboard.getNumber("Drive p", 0)){
        //     previousP = SmartDashboard.getNumber("Drive p", 0);
        //     leftMaster.config_kP(0, previousP);
        //     rightMaster.config_kP(0, previousP);
        // }

        SmartDashboard.putNumber("left drive", Conversions.falconToMPS(
            leftMaster.getSelectedSensorVelocity(), 
            Constants.Drive.wheelCircumference, 
            Constants.Drive.gearRatio));

        SmartDashboard.putNumber("right drive", Conversions.falconToMPS(
            rightMaster.getSelectedSensorVelocity(), 
            Constants.Drive.wheelCircumference, 
            Constants.Drive.gearRatio));

        SmartDashboard.putNumber("yaw", getYaw().getDegrees());
        SmartDashboard.putNumber("x odo", m_odometry.getPoseMeters().getX());
        SmartDashboard.putNumber("y odo", m_odometry.getPoseMeters().getY());

        SmartDashboard.putNumber("LDrive Amps", leftMaster.getSupplyCurrent());
        SmartDashboard.putNumber("RDrive Amps", rightMaster.getSupplyCurrent());

        SmartDashboard.putNumber("LDrive Temp", leftMaster.getTemperature());
        SmartDashboard.putNumber("RDrive Temp", rightMaster.getTemperature());



        
        /* Falcon Dashboard Setup*/
        NetworkTableInstance inst = NetworkTableInstance.getDefault();
        NetworkTable table = inst.getTable("Live_Dashboard");

        /* PoseEstimator Values */
        NetworkTableEntry robotX = table.getEntry("robotX");
        NetworkTableEntry robotY = table.getEntry("robotY");
        NetworkTableEntry robotHeading = table.getEntry("robotHeading");

        robotX.setDouble(Units.metersToFeet(m_odometry.getPoseMeters().getX()));
        robotY.setDouble(Units.metersToFeet(m_odometry.getPoseMeters().getY()));
        robotHeading.setDouble(m_odometry.getPoseMeters().getRotation().getRadians());
        table.getEntry("isFollowingPath").setBoolean(true);
    }
}
