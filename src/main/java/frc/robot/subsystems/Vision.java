package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.util.Limelight;
import frc.robot.Constants;
import frc.robot.States;
import frc.robot.States.ShooterStates;

public class Vision extends SubsystemBase {
    public Limelight limelight;

    public Vision() {
        limelight = new Limelight(
            Constants.Vision.limelightHeight, 
            Constants.Vision.limelightAngle, 
            Constants.Vision.goalHeight
        );
    }    
    
    @Override
    public void periodic(){
        if (States.shooterState == ShooterStates.preShoot){
            limelight.ledState(3);
        }
        else{
            limelight.ledState(1);
        }
        
        SmartDashboard.putNumber("LLDistance", limelight.getDistance().getNorm());
    }

}