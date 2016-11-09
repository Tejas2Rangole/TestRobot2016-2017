package org.usfirst.frc.team948.robot.commands;

import org.usfirst.frc.team948.robot.OI;

public class  ManualDriveStraight {
	
	public ManualDriveStraight(){
		//Need command base
		requires(CommandBase.drive);
	}
	protected void initialize(){
		
	}
	protected void execute(){
		//Need drive subsystem
		drive.rawTankDrive(OI.getLeftJoystick(), OI.getRightJoystick());
	}
	protected boolean isFinished(){
		return false;
	}
	protected void end(){
		drive.rawStop();
	}
	protected void interrupted(){
		end();
	}
	
	

}
