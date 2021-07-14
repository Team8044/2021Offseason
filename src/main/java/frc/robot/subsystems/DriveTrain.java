package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.Controllers.WPI_LazyTalonFX;
import frc.lib.math.Boundaries;
import frc.lib.math.Conversions;
import frc.robot.Constants;

public class DriveTrain extends SubsystemBase {

    private WPI_LazyTalonFX leftMaster;
    private WPI_LazyTalonFX leftSlave;
    private WPI_LazyTalonFX rightMaster;
    private WPI_LazyTalonFX rightSlave;

    private DifferentialDrive m_robotDrive;
    private DifferentialDriveOdometry m_odometry;
    private SimpleMotorFeedforward driveFF;

    private AHRS gyro;

    // private double previousP = 0;

    public DriveTrain() {
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

        SmartDashboard.putNumber("Left Amps", leftMaster.getSupplyCurrent());
        SmartDashboard.putNumber("Right Amps", rightMaster.getSupplyCurrent());

        SmartDashboard.putNumber("LDrive Temp", leftMaster.getTemperature());
        SmartDashboard.putNumber("RDrive Temp", rightMaster.getTemperature());


    }
}
