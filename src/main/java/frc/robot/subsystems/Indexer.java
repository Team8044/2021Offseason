package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.Controllers.LazyVictorSPX;
import frc.robot.Constants;

public class Indexer extends SubsystemBase {
  private VictorSPX indexerMotor1;
  private LazyVictorSPX indexerMotor2;

  public Indexer() {
    indexerMotor1 = new LazyVictorSPX(Constants.Indexer.indexerMotor1);
    indexerMotor2 = new LazyVictorSPX(Constants.Indexer.indexerMotor2);
  }

  public void setPower(double power) {
    indexerMotor1.set(ControlMode.PercentOutput, power);
    indexerMotor2.set(ControlMode.PercentOutput, power);
  }

  @Override
  public void periodic() {
    
  }
}
