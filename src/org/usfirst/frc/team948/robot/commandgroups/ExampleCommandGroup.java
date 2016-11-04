
package org.usfirst.frc.team948.robot.commands;

import edu.wpi.first.wpilibj.command.Command;

import org.usfirst.frc.team948.robot.Robot;

/**
 * 
 */

//WIP look at at your own risk, possibly totaly wrong
public class ExampleCommandGroup extends CommandGroup {
	//This is run when a new command group is created, no other specific functions in command groups are important to the WPI libraries
	public ExampleCommandGroup(){
		//Uses addSequential to schedule one sequential running of ExampleCommand
		addSequential(new ExampleCommand()); //Command 1
		//Uses addParallel and add Sequential to schedule the parellel running of two more ExampleCommands after command one
		addParallel(new ExampleCommand()); //Command 2
		addSequential(new ExampleCommand()); //Command 3
		//After the last command (or all of the last parrallel batch of commands) are finished running the command group is done
	}

}
