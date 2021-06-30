package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.Intake;

public class IntakePistonControl extends InstantCommand{
    private Intake m_Intake;
	private boolean extended;

	public IntakePistonControl(Intake m_Intake, Boolean extended){
        this.m_Intake = m_Intake;
		this.extended = extended;
	}
	
    @Override
	public void initialize() {
		addRequirements(m_Intake);
	}

    @Override
	public void execute() {
		m_Intake.setPiston(extended);
	}

	@Override
	public void end(boolean interrupted){
	}
 
}
