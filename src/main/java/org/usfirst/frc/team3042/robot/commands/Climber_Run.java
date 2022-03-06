package org.usfirst.frc.team3042.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.util.sendable.SendableRegistry;

import org.usfirst.frc.team3042.lib.Log;
import org.usfirst.frc.team3042.robot.Robot;
import org.usfirst.frc.team3042.robot.RobotMap;
import org.usfirst.frc.team3042.robot.subsystems.Climber;

/** Climber_Run *******************************************************************
 * Sets power to the climber */
public class Climber_Run extends CommandBase {
	/** Configuration Constants ***********************************************/
	private static final Log.Level LOG_LEVEL = RobotMap.LOG_CLIMBER;
	private static final double POWER = RobotMap.CLIMBER_POWER;
	
	/** Instance Variables ****************************************************/
  	Climber climber = Robot.climber;
	Log log = new Log(LOG_LEVEL, SendableRegistry.getName(climber));
	int direction;

	/** Climber Run ****************************************************************
	 * Required subsystems will cancel commands when this command is run. */
	public Climber_Run(int direction) {
		log.add("Constructor", Log.Level.TRACE);
    	this.direction = direction;
		addRequirements(climber);
	}

	/** initialize ************************************************************
	 * Called just before this Command runs the first time */
	public void initialize() {
		log.add("Initialize", Log.Level.TRACE);
		if(climber.isRetracted) {
			climber.setPower(POWER * direction);
		} else {
			climber.stop();
		}
	}

	/** execute ***************************************************************
	 * Called repeatedly when this Command is scheduled to run */
	public void execute() {}
	
	/** isFinished ************************************************************	
	 * Make this return true when this Command no longer needs to run execute() */
	public boolean isFinished() {
		return false;
	}

	// Called once the command ends or is interrupted.
	public void end(boolean interrupted) {
		log.add("End", Log.Level.TRACE);
		climber.stop();
	}
}