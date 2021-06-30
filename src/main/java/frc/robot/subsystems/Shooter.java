package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.revrobotics.ControlType;
import com.revrobotics.CANSparkMax.SoftLimitDirection;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.Controllers.LazySparkMAX;
import frc.lib.Controllers.LazyTalonFX;
import frc.lib.math.Conversions;
import frc.lib.util.Interpolatable;
import frc.lib.util.InterpolatableTreeMap;
import frc.lib.util.Limelight;
import frc.robot.Constants;
import frc.robot.States;
import frc.robot.States.ShooterStates;

public class Shooter extends SubsystemBase {
    private LazyTalonFX shooterMaster;
    private LazyTalonFX shooterSlave;
    private LazySparkMAX angleMaster;

    private InterpolatableTreeMap<Double> shooterMap = new InterpolatableTreeMap<>();
    private InterpolatableTreeMap<Double> angleMap = new InterpolatableTreeMap<>();
    private Limelight limelight;

    private PIDController angleController;

    public Shooter(Vision m_Vision) {
        limelight = m_Vision.limelight;
        shooterMaster = new LazyTalonFX(Constants.Shooter.shooterMasterConstats);
        shooterSlave = new LazyTalonFX(Constants.Shooter.shooterSlaveConstants, shooterMaster);
        shooterMaster.configPID(Constants.Shooter.shooterPID);

        angleMaster = new LazySparkMAX(Constants.Shooter.angleMasterConstants);

        angleMaster.setSoftLimit(SoftLimitDirection.kForward, Constants.Shooter.angleForwardLimit);
        angleMaster.setSoftLimit(SoftLimitDirection.kReverse, Constants.Shooter.angleReverseLimit);
        angleMaster.enableSoftLimit(SoftLimitDirection.kForward, false);
        angleMaster.enableSoftLimit(SoftLimitDirection.kReverse, false);

        angleController = new PIDController(
            Constants.Shooter.anglePID.kP,
            Constants.Shooter.anglePID.kI,
            Constants.Shooter.anglePID.kD
        );

        /* Creating Interpolatable TreeMaps for Shooter RPM and Angle with Shooter Map Values */
        for (int i = 0; i < Constants.Shooter.shooterMap.length; ++i) {
            shooterMap.set(Constants.Shooter.shooterMap[i][0], Interpolatable.interDouble(Constants.Shooter.shooterMap[i][1]));
            angleMap.set(Constants.Shooter.shooterMap[i][0], Interpolatable.interDouble(Constants.Shooter.shooterMap[i][2]));
        }

        if(Constants.Shooter.calibrationMode){
            SmartDashboard.putNumber("Shooter RPM Calib", 0);
            SmartDashboard.putNumber("Shooter Angle Calib", 0);
        }
    }

    public double getShooterRPM(){
        return Conversions.falconToRPM(shooterMaster.getSelectedSensorVelocity(), Constants.Shooter.shooterGearRatio);
    }

    public void setShooterRPM(double shooterRPM){
        double falconVelocity = Conversions.RPMToFalcon(shooterRPM, Constants.Shooter.shooterGearRatio);
        shooterMaster.set(ControlMode.Velocity, falconVelocity);
    }

    public double getShooterAngle(){
        return angleMaster.getPosition();
    }

    public void setShooterAngle(double angle){
        double shooterAngle = (angle < Constants.Shooter.angleReverseLimit) ? Constants.Shooter.angleReverseLimit : angle;
        shooterAngle = (shooterAngle > Constants.Shooter.angleForwardLimit) ? Constants.Shooter.angleForwardLimit : angle;
        double demand = angleController.calculate(angleMaster.getPosition(), shooterAngle);
        angleMaster.set(ControlType.kDutyCycle, demand);
    }

    private boolean resetShooterTilt(){
        if(angleMaster.getOutputCurrent() < 5.0){
            angleMaster.set(ControlType.kDutyCycle, -0.1);
            return false;
        } else{
            angleMaster.set(ControlType.kDutyCycle, 0);
            angleMaster.setPosition(0);
            angleMaster.enableSoftLimit(SoftLimitDirection.kForward, false);
            angleMaster.enableSoftLimit(SoftLimitDirection.kReverse, false);
            return true;
        }
    }

    @Override
    public void periodic() {
        switch(States.shooterState){
            case notCalibrated:        
                if(resetShooterTilt()){
                    States.shooterState = ShooterStates.disabled;
                }
                shooterMaster.set(ControlMode.PercentOutput, 0);
                break;
            case disabled:
                shooterMaster.set(ControlMode.PercentOutput, 0);
                // setShooterAngle(0.0);
                angleMaster.set(ControlType.kDutyCycle, 0.0);
                break;
            case preShoot:
                setShooterRPM(Constants.Shooter.calibrationMode ? SmartDashboard.getNumber("Shooter RPM Calib", 0) : shooterMap.get(limelight.getDistance().getNorm()));
                setShooterAngle(Constants.Shooter.calibrationMode ? SmartDashboard.getNumber("Shooter Angle Calib", 0) : angleMap.get(limelight.getDistance().getNorm()));
                break;
        }

        SmartDashboard.putNumber("Shooter RPM", getShooterRPM());
        SmartDashboard.putNumber("Shooter Angle", getShooterAngle());
    }
}
