
package org.usfirst.frc.team948.robot.commands;

import edu.wpi.first.wpilibj.command.Command;

import org.usfirst.frc.team948.robot.Robot;

/**
 * 
 */

//WIP look at at your own risk, possibly totaly wrong
public class ExampleCommandGroup extends CommandGroup {
	public ExampleCommandGroup(){
		//Uses addSequential to schedule one sequential running of ExampleCommand
		addSequential(new ExampleCommand()); //Command 1
		//Uses addParallel and add Sequential to schedule the parellel running of two more ExampleCommands after command one
		addParallel(new ExampleCommand()); //Command 2
		addSequential(new ExampleCommand()); //Command 3
	}

}
