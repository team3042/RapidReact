package org.usfirst.frc.team3042.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.util.sendable.SendableRegistry;

import org.usfirst.frc.team3042.lib.Log;
import org.usfirst.frc.team3042.robot.Robot;
import org.usfirst.frc.team3042.robot.RobotMap;
import org.usfirst.frc.team3042.robot.subsystems.Conveyor;

/** Conveyor_Advance *******************************************************************
 * Auto advance the conveyor until the limit switch detects a ball */
public class Conveyor_Advance extends CommandBase {
	/** Configuration Constants ***********************************************/
	private static final Log.Level LOG_LEVEL = RobotMap.LOG_CONVEYOR;
	private static final double POWER = RobotMap.CONVEYOR_ADVANCE_POWER;
  
	/** Instance Variables ****************************************************/
  	Conveyor conveyor = Robot.conveyor;
	Log log = new Log(LOG_LEVEL, SendableRegistry.getName(conveyor));

	/** Conveyor ****************************************************************
	 * Required subsystems will cancel commands when this command is run. */
	public Conveyor_Advance() {
		log.add("Constructor", Log.Level.TRACE);
		addRequirements(conveyor);	
	}

	/** initialize ************************************************************
	 * Called just before this Command runs the first time */
	public void initialize() {
		log.add("Initialize", Log.Level.TRACE);
		conveyor.setPower(POWER);
	}

	/** execute ***************************************************************
	 * Called repeatedly when this Command is scheduled to run */
	public void execute() {}

	/** isFinished ************************************************************	
	 * Make this return true when this Command no longer needs to run execute() */
	public boolean isFinished() {
		return conveyor.getLimitStatus();
	}

	// Called once the command ends or is interrupted.
	public void end(boolean interrupted) {
		log.add("End", Log.Level.TRACE);
		conveyor.stop();
	}
}