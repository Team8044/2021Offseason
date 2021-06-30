package frc.robot.commands;

import frc.robot.subsystems.Indexer;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class IndexerControl extends CommandBase {
  private Indexer m_Indexer;
  private double power;

  public IndexerControl(Indexer m_Indexer, double power) {
    this.m_Indexer = m_Indexer;
    this.power = power;
  }

  @Override
  public void initialize() {
    addRequirements(m_Indexer);
  }

  @Override
  public void execute() {
    m_Indexer.setPower(power);
  }

  @Override
  public void end(boolean interrupted) {
    m_Indexer.setPower(0);
  }
}
