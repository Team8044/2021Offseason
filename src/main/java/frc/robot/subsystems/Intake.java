package frc.robot.subsystems;

import com.revrobotics.ControlType;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static edu.wpi.first.wpilibj.DoubleSolenoid.Value.kForward;
import static edu.wpi.first.wpilibj.DoubleSolenoid.Value.kReverse;

import frc.lib.Controllers.LazySparkMAX;
import frc.robot.Constants;

public class Intake extends SubsystemBase {
  private LazySparkMAX intake;
  private DoubleSolenoid intakePiston;

  public Intake() {
    intake = new LazySparkMAX(Constants.Intake.motorConstants);

    intakePiston = new DoubleSolenoid(Constants.Intake.pistonExtend, Constants.Intake.pistonRetract);
  }
    
  public void setPower(double power){
      intake.set(ControlType.kDutyCycle, power);
  }

  public void setPiston(boolean extended){
      intakePiston.set(extended ? kForward : kReverse);
  }

  @Override
  public void periodic() {
    
  }
}
