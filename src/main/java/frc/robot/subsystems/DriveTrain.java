package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
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

    private AHRS gyro;

    public DriveTrain() {
        leftMaster = new WPI_LazyTalonFX(Constants.Drive.leftMaster);
        leftSlave = new WPI_LazyTalonFX(Constants.Drive.leftSlave, leftMaster);
        rightMaster = new WPI_LazyTalonFX(Constants.Drive.rightMaster);
        rightSlave = new WPI_LazyTalonFX(Constants.Drive.rightSlave, rightMaster);

        leftMaster.configPID(Constants.Drive.drivePID);
        rightMaster.configPID(Constants.Drive.drivePID);

        gyro = new AHRS();

        m_robotDrive = new DifferentialDrive(leftMaster, rightMaster);
        m_odometry = new DifferentialDriveOdometry(getYaw());
    }

    /* For standard Teleop Drive */
    public void drive(double speed, double rotation){
        m_robotDrive.arcadeDrive(speed, rotation);
    }

    /* Used for RamseteController for Following Auto Paths */
    public void setWheelState(double leftSpeed, double rightSpeed){
        SimpleMotorFeedforward driveFF = Constants.Drive.driveFF;

        double leftDemand = Conversions.MPSToFalcon(leftSpeed, Constants.Drive.wheelCircumference, Constants.Drive.gearRatio);
        double rightDemand = Conversions.MPSToFalcon(rightSpeed, Constants.Drive.wheelCircumference, Constants.Drive.gearRatio);

        double leftFF = (driveFF.calculate(leftDemand) / 12.0); //Divide by 12 because CTRE is in percent not voltage
        double rightFF = (driveFF.calculate(rightDemand) / 12.0); //Divide by 12 because CTRE is in percent not voltage

        leftMaster.set(ControlMode.Velocity, leftDemand, DemandType.ArbitraryFeedForward, leftFF);
        rightMaster.set(ControlMode.Velocity, rightDemand, DemandType.ArbitraryFeedForward, rightFF);
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
    }
}
