package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.Controllers.LazyVictorSPX;
import frc.robot.Constants;

public class Indexer extends SubsystemBase {
  private LazyVictorSPX indexerMotorRight;
  private LazyVictorSPX indexerMotorLeft;

  public Indexer() {
    indexerMotorRight = new LazyVictorSPX(Constants.Indexer.indexerMotorRight);
    indexerMotorLeft = new LazyVictorSPX(Constants.Indexer.indexerMotorLeft);
  }

  public void setPower(double power) {
    indexerMotorRight.set(ControlMode.PercentOutput, power);
    indexerMotorLeft.set(ControlMode.PercentOutput, power);
  }

  @Override
  public void periodic() {
    
  }
}
