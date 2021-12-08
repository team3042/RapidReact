package org.usfirst.frc.team3042.robot.commands.autonomous.helperCommands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;

/** Wait *******************************************************
 * Waits for a specified number of seconds. Useful for autonomous command groups! */
public class Wait extends Command {

	int duration;
	Timer timer = new Timer();

	public Wait(int time) {
		duration = time;
	}

	protected void initialize() {
		timer.reset();
		timer.start();
	}

	protected void execute() {}
	
	protected boolean isFinished() {
		return timer.get() >= duration;
	}
	
	protected void end() {
		timer.reset();
	}

	protected void interrupted() {
		timer.reset();
	}
}