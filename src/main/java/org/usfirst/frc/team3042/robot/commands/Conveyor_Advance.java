package org.usfirst.frc.team3042.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.util.sendable.SendableRegistry;

import org.usfirst.frc.team3042.lib.Log;
import org.usfirst.frc.team3042.robot.Robot;
import org.usfirst.frc.team3042.robot.RobotMap;
import org.usfirst.frc.team3042.robot.subsystems.Conveyor;

/** Conveyor_Advance *******************************************************************
 * Auto advance the conveyor by a set increment whenever the sensor detects a ball */
public class Conveyor_Advance extends Command {
	/** Configuration Constants ***********************************************/
	private static final Log.Level LOG_LEVEL = RobotMap.LOG_CONVEYOR;
	private static final double POWER = RobotMap.CONVEYOR_POWER;
  
	
	/** Instance Variables ****************************************************/
  Conveyor conveyor = Robot.conveyor;
	Log log = new Log(LOG_LEVEL, SendableRegistry.getName(conveyor));
  int direction;

	/** Conveyor ****************************************************************
	 * Required subsystems will cancel commands when this command is run. */
	public Conveyor_Advance(int direction) {
		log.add("Constructor", Log.Level.TRACE);
    this.direction = direction;
		requires(conveyor);
	}

	/** initialize ************************************************************
	 * Called just before this Command runs the first time */
	protected void initialize() {
		log.add("Initialize", Log.Level.TRACE);
		conveyor.setPower(POWER * direction);
	}

	/** execute ***************************************************************
	 * Called repeatedly when this Command is scheduled to run */
	protected void execute() {}
	
	/** isFinished ************************************************************	
	 * Make this return true when this Command no longer needs to run execute() */
	protected boolean isFinished() {
		return false;
	}
	
	/** end *******************************************************************
	 * Called once after isFinished returns true */
	protected void end() {
		log.add("End", Log.Level.TRACE);
		conveyor.stop();
	}

	/** interrupted ***********************************************************
	 * Called when another command which requires one or more of the same
	 * subsystems is scheduled to run */
	protected void interrupted() {
		log.add("Interrupted", Log.Level.TRACE);
		conveyor.stop();
	}
}