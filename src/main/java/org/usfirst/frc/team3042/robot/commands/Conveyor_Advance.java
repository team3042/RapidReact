package org.usfirst.frc.team3042.robot.commands;

import edu.wpi.first.wpilibj.DigitalInput;
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
	private static final double POWER = RobotMap.CONVEYOR_POWER * 0.5;
	private static final int DIO_LIMITSWITCH = RobotMap.DIO_LIMITSWITCH_CONVEYOR;
  
	/** Instance Variables ****************************************************/
  	Conveyor conveyor = Robot.conveyor;
	Log log = new Log(LOG_LEVEL, SendableRegistry.getName(conveyor));
	DigitalInput limit = new DigitalInput(DIO_LIMITSWITCH);

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
	public void execute() {
		if(limit.get()) {
			conveyor.stop();
		}
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