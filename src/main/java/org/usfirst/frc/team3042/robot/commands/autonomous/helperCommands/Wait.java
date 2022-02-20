package org.usfirst.frc.team3042.robot.commands.autonomous.helperCommands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;

/** Wait *******************************************************
 * Waits for a specified number of seconds. Useful for autonomous command groups! */
public class Wait extends CommandBase {

	double duration;
	Timer timer = new Timer();

	public Wait(double time) {
		duration = time;
	}

	public void initialize() {
		timer.reset();
		timer.start();
	}

	public void execute() {}
	
	public boolean isFinished() {
		return timer.get() >= duration;
	}
	
	protected void end() {
		timer.reset();
	}

	protected void interrupted() {
		timer.reset();
	}
}