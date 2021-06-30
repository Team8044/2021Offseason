package frc.robot.subsystems;

import com.revrobotics.ControlType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.Controllers.LazySparkMAX;
import frc.robot.Constants;

public class Kicker extends SubsystemBase {
  private LazySparkMAX kicker;

  public Kicker() {
    kicker = new LazySparkMAX(Constants.Kicker.motorConstants);

  }

  public void setPower(double power){
    kicker.set(ControlType.kDutyCycle, power);
  }

  @Override
  public void periodic() {
    
  }
}
