package frc.robot;

import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj.util.Units;
import frc.lib.Controllers.SparkConstants;
import frc.lib.Controllers.TalonConstants;
import frc.lib.math.PIDGains;

public final class Constants {

    public static final class Drive {
        /* Motors */
		public static final TalonConstants leftMaster = 
            new TalonConstants(1, talonCurrentLimit.supplyCurLim40, NeutralMode.Brake, InvertType.None);
    
        public static final TalonConstants leftSlave = 
            new TalonConstants(2, talonCurrentLimit.supplyCurLim40, NeutralMode.Brake, InvertType.FollowMaster);

        public static final TalonConstants rightMaster = 
            new TalonConstants(3, talonCurrentLimit.supplyCurLim40, NeutralMode.Brake, InvertType.None);
            
        public static final TalonConstants rightSlave = 
            new TalonConstants(4, talonCurrentLimit.supplyCurLim40, NeutralMode.Brake, InvertType.FollowMaster);

        
        /* Drivebase Constants */
		public static final double gearRatio = (9.47 / 1.0);
		public static final double wheelDiameter = Units.inchesToMeters(6.0);
        public static final double wheelCircumference = wheelDiameter * Math.PI;

        public static final double trackWidth = Units.inchesToMeters(25.27 - 2.063);
        public static final DifferentialDriveKinematics driveKinematics = 
            new DifferentialDriveKinematics(trackWidth);

        /* Drive Motor Feed Forward Characterization Values */
        public static final double driveKS = (0.648);
        public static final double driveKV = (2.09);
        public static final double driveKA = (0.286);
        public static final SimpleMotorFeedforward driveFF = 
            new SimpleMotorFeedforward(driveKS, driveKV, driveKA);

        public static final PIDGains drivePID = new PIDGains(0.1, 0.0, 0.0, 0.0); //TODO
        
		public static final boolean invertGyro = false;
    }

    public static final class Intake {
        public static final SparkConstants motorConstants =
            new SparkConstants(10, MotorType.kBrushed, 25, IdleMode.kCoast, true);

        public static final int pistonExtend = 0;
        public static final int pistonRetract = 1;
    }

    public static final class Indexer {
        public static final TalonConstants indexerMotor1 =
            new TalonConstants(7, talonCurrentLimit.supplyCurLim30, NeutralMode.Coast, InvertType.None);

        public static final TalonConstants indexerMotor2 =
            new TalonConstants(8, talonCurrentLimit.supplyCurLim30, NeutralMode.Coast, InvertType.None);
    }

    public static final class Kicker {
        public static final SparkConstants motorConstants =
            new SparkConstants(12, MotorType.kBrushless, 25, IdleMode.kCoast, true);
    }

    public static final class Shooter {
        public static final TalonConstants shooterMasterConstats = 
            new TalonConstants(5, talonCurrentLimit.supplyCurLim40, NeutralMode.Coast, InvertType.InvertMotorOutput);
            
        public static final TalonConstants shooterSlaveConstants = 
            new TalonConstants(6, talonCurrentLimit.supplyCurLim40, NeutralMode.Coast, InvertType.OpposeMaster);    

        public static final SparkConstants angleMotorConstants =
            new SparkConstants(9, MotorType.kBrushless, 60, IdleMode.kCoast, false);

        public static final PIDGains shooterPID = new PIDGains(0.1, 0.0, 0.0, 0.046976);
        public static final PIDGains anglePID = new PIDGains(0.175, 0.0, 0.0, 0.0);

        public static final double shooterGearRatio = (1.0 / 1.0);
        public static final double angleGearRatio = (1.0 / 1.0);

        public static final float angleForwardLimit = 12;
        public static final float angleReverseLimit = 0;

        public static final boolean calibrationMode = false;

        /* Shooter Tuned Constants */
        public static final double[][] shooterMap = // TODO
        // {distance, shooterRPM, shooterAngle}
        {
             {3.0, 3200.0, 10.0},
             {4.0, 3200.0, 9.0},
             {5.0, 3000.0, 8.0},
             {6.0, 3000.0, 7.0},
             {7.0, 3100.0, 6.5},
             {9.0, 3100.0, 6.0},
        };
    }

    

    public static final class Vision {
        public static final double goalHeight = Units.inchesToMeters(81.0 + (17.0 / 2.0));

        public static final double limelightHeight = Units.inchesToMeters(21.0);
        public static final Rotation2d limelightAngle = Rotation2d.fromDegrees(9.0);
    }

    public static final class talonCurrentLimit {
        public static final SupplyCurrentLimitConfiguration supplyCurLim40 = 
            new SupplyCurrentLimitConfiguration(true, 35, 60, 0.1);
            
        public static final SupplyCurrentLimitConfiguration supplyCurLim30 = 
            new SupplyCurrentLimitConfiguration(true, 25, 40, 0.1);
    }

    public static final class AutoConstants {
        public static final double maxSpeed = 3; //MPS TODO
        public static final double maxAcelleration = 3; //MPSS (meters per second squared) TODO

        // Reasonable baseline values for a RAMSETE follower in units of meters and seconds
        public static final double kRamseteB = 2;
        public static final double kRamseteZeta = 0.7;

        // Create a voltage constraint to ensure we don't accelerate too fast
        public static final DifferentialDriveVoltageConstraint autoVoltageConstraint =
            new DifferentialDriveVoltageConstraint(Drive.driveFF, Drive.driveKinematics, 10);

        // Config for Trajectory Generation
        public static final TrajectoryConfig trajConfig =
            new TrajectoryConfig(maxSpeed, maxAcelleration)
                .setKinematics(Constants.Drive.driveKinematics)
                .addConstraint(Constants.AutoConstants.autoVoltageConstraint);
    }

}
