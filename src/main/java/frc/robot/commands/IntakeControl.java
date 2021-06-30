package frc.robot.commands;

import frc.robot.subsystems.Intake;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class IntakeControl extends CommandBase {
  private Intake m_Intake;
  private double power;

  public IntakeControl(Intake m_Intake, double power) {
    this.m_Intake = m_Intake;
    this.power = power;
  }

  @Override
  public void initialize() {
    addRequirements(m_Intake);
  }

  @Override
  public void execute() {
    m_Intake.setPower(power);
  }

  @Override
  public void end(boolean interrupted) {
    m_Intake.setPower(0);
  }
}
