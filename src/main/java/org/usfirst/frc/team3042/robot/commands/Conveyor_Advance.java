package org.usfirst.frc.team3042.robot.commands;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.util.sendable.SendableRegistry;

import org.usfirst.frc.team3042.lib.Log;
import org.usfirst.frc.team3042.robot.Robot;
import org.usfirst.frc.team3042.robot.RobotMap;
import org.usfirst.frc.team3042.robot.subsystems.Conveyor;

/** Conveyor_Advance *******************************************************************
 * Auto advance the conveyor by a set increment whenever the switch detects a ball in the conveyor */
public class Conveyor_Advance extends CommandBase {
	/** Configuration Constants ***********************************************/
	private static final Log.Level LOG_LEVEL = RobotMap.LOG_CONVEYOR;
	private static final double POWER = RobotMap.CONVEYOR_POWER;
	private static final double duration = RobotMap.CONVEYOR_ADVANCE_DURATION;
	private static final int DIO_LIMITSWITCH = RobotMap.DIO_LIMITSWITCH;
  
	/** Instance Variables ****************************************************/
  	Conveyor conveyor = Robot.conveyor;
	Log log = new Log(LOG_LEVEL, SendableRegistry.getName(conveyor));
	DigitalInput limit = new DigitalInput(DIO_LIMITSWITCH);
	Timer timer = new Timer();

	/** Conveyor ****************************************************************
	 * Required subsystems will cancel commands when this command is run. */
	public Conveyor_Advance() {
		log.add("Constructor", Log.Level.TRACE);
		addRequirements(conveyor);	}

	/** initialize ************************************************************
	 * Called just before this Command runs the first time */
	public void initialize() {
		log.add("Initialize", Log.Level.TRACE);
		timer.reset();
	}

	/** execute ***************************************************************
	 * Called repeatedly when this Command is scheduled to run */
	public void execute() {
		if(limit.get()){
			conveyor.setPower(POWER);
			timer.start();
		}
		if(timer.get() >= duration){
			conveyor.stop();
			timer.stop();
			timer.reset();
		}
	}
	
	/** isFinished ************************************************************	
	 * Make this return true when this Command no longer needs to run execute() */
	public boolean isFinished() {
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