package frc.robot.commands;

import frc.robot.subsystems.Kicker;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class KickerControl extends CommandBase {
  private Kicker m_Kicker;
  private double power;

  public KickerControl(Kicker m_Kicker, double power) {
    this.m_Kicker = m_Kicker;
    this.power = power;
  }

  @Override
  public void initialize() {
    addRequirements(m_Kicker);
  }

  @Override
  public void execute() {
    m_Kicker.setPower(power);
  }

  @Override
  public void end(boolean interrupted) {
    m_Kicker.setPower(0);
  }
}
