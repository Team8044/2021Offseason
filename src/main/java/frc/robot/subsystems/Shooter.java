package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.revrobotics.ControlType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
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

public class Shooter extends SubsystemBase {
    private LazyTalonFX shooterMaster;
    private LazyTalonFX shooterSlave;
    private LazySparkMAX angleMotor;

    private DutyCycleEncoder angleEncoder;

    private InterpolatableTreeMap<Double> shooterMap = new InterpolatableTreeMap<>();
    private InterpolatableTreeMap<Double> angleMap = new InterpolatableTreeMap<>();
    private Limelight limelight;

    private PIDController angleController;

    public Shooter(Vision m_Vision) {
        limelight = m_Vision.limelight;
        shooterMaster = new LazyTalonFX(Constants.Shooter.shooterMasterConstats);
        shooterSlave = new LazyTalonFX(Constants.Shooter.shooterSlaveConstants);

        shooterMaster.configPID(Constants.Shooter.shooterPID);
        shooterSlave.follow(shooterMaster);

        angleMotor = new LazySparkMAX(Constants.Shooter.angleMotorConstants);
        angleEncoder = new DutyCycleEncoder(new DigitalInput(Constants.Shooter.angleEncoderPort));

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
        double position = angleEncoder.get() * 360;
        position = 360 - position;
        position = position - Constants.Shooter.angleEncoderOffset;
        position = Math.round(position * 100.0) / 100.0; // Rounding to 2 decimal places
        return position;
    }

    public void setShooterAngle(double angle){
        double finalAngle = angle;
        if (angle < Constants.Shooter.angleReverseLimit){
            finalAngle = Constants.Shooter.angleReverseLimit;
        }
        else if (angle > Constants.Shooter.angleForwardLimit) {
            finalAngle = Constants.Shooter.angleForwardLimit;
        }
        double demand = angleController.calculate(getShooterAngle(), finalAngle);
        angleMotor.set(ControlType.kDutyCycle, demand);
    }

    @Override
    public void periodic() {
        switch(States.shooterState){
            case disabled:
                shooterMaster.set(ControlMode.PercentOutput, 0);
                angleMotor.set(ControlType.kDutyCycle, 0);
                break;
                
            case preShoot:
                if (Constants.Shooter.calibrationMode){
                    setShooterRPM(SmartDashboard.getNumber("Shooter RPM Calib", 0));
                    setShooterAngle(SmartDashboard.getNumber("Shooter Angle Calib", 0));
                } else{
                    setShooterRPM(shooterMap.get(limelight.getDistance().getNorm()));
                    setShooterAngle(angleMap.get(limelight.getDistance().getNorm()));
                }
                break;
        }

        SmartDashboard.putNumber("Shooter RPM", getShooterRPM());
        SmartDashboard.putNumber("Shooter Angle", getShooterAngle());

        SmartDashboard.putNumber("MShooter Temp", shooterMaster.getTemperature());
        SmartDashboard.putNumber("SShooter Temp", shooterSlave.getTemperature());
    }
}
