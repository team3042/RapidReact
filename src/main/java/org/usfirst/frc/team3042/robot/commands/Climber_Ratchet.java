package org.usfirst.frc.team3042.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.util.sendable.SendableRegistry;

import org.usfirst.frc.team3042.lib.Log;
import org.usfirst.frc.team3042.robot.Robot;
import org.usfirst.frc.team3042.robot.RobotMap;
import org.usfirst.frc.team3042.robot.subsystems.Climber;

/** Climber_Ratchet *******************************************************************
 * Extends or retracts the climber */
public class Climber_Ratchet extends CommandBase {
	/** Configuration Constants ***********************************************/
	private static final Log.Level LOG_LEVEL = RobotMap.LOG_CLIMBER;
	
	/** Instance Variables ****************************************************/
	Climber climber = Robot.climber;
	Log log = new Log(LOG_LEVEL, SendableRegistry.getName(climber));
	boolean isRetracted = true;

	/** Climber ****************************************************************
	 * Required subsystems will cancel commands when this command is run. */
	public Climber_Ratchet() {
		log.add("Constructor", Log.Level.TRACE);
		addRequirements(climber);
	}

	/** initialize ************************************************************
	 * Called just before this Command runs the first time */
	public void initialize() {
		log.add("Initialize", Log.Level.TRACE);
		if (isRetracted == true) {
    		climber.extend();
			isRetracted = false;
    	}
    	else {
    		climber.retract();
			isRetracted = true;
    	}
	}

	/** execute ***************************************************************
	 * Called repeatedly when this Command is scheduled to run */
	public void execute() {}
	
	/** isFinished ************************************************************	
	 * Make this return true when this Command no longer needs to run execute() */
	public boolean isFinished() {
		return true;
	}
	
	/** end *******************************************************************
	 * Called once after isFinished returns true */
	protected void end() {
		log.add("End", Log.Level.TRACE);
	}

	/** interrupted ***********************************************************
	 * Called when another command which requires one or more of the same
	 * subsystems is scheduled to run */
	protected void interrupted() {
		log.add("Interrupted", Log.Level.TRACE);
	}
}