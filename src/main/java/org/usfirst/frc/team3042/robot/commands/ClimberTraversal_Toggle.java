package org.usfirst.frc.team3042.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.util.sendable.SendableRegistry;

import org.usfirst.frc.team3042.lib.Log;
import org.usfirst.frc.team3042.robot.Robot;
import org.usfirst.frc.team3042.robot.RobotMap;
import org.usfirst.frc.team3042.robot.subsystems.ClimberTraversal;

/** ClimberTraversal_Toggle *******************************************************************
 * Toggles the traversal climbing hooks */
public class ClimberTraversal_Toggle extends CommandBase {
	/** Configuration Constants ***********************************************/
	private static final Log.Level LOG_LEVEL = RobotMap.LOG_CLIMBER_TRAVERSAL;
	private static final double goalPos = RobotMap.TRAVERSAL_GOAL_POSITION;	
	private static final double POWER = RobotMap.TRAVERSAL_CLIMBER_POWER;

	/** Instance Variables ****************************************************/
	ClimberTraversal traversal = Robot.traversal;
	Log log = new Log(LOG_LEVEL, SendableRegistry.getName(traversal));
	private boolean retracting;

	/** ClimberTraversal_Climb ****************************************************************
	 * Required subsystems will cancel commands when this command is run. */
	public ClimberTraversal_Toggle() {
		log.add("Constructor", Log.Level.TRACE);
		addRequirements(traversal);	
	}

	/** initialize ************************************************************
	 * Called just before this Command runs the first time */
	public void initialize() {
		log.add("Initialize", Log.Level.TRACE);

		if(traversal.isRetracted() == true) {
			retracting = false;
		}
		else if (traversal.isRetracted() == false) {
			retracting = true;
		}

		traversal.toggle();
	}

	/** execute ***************************************************************
	 * Called repeatedly when this Command is scheduled to run */
	public void execute() {		
		if (!retracting) {
			traversal.setPower(POWER);
		}
		else if (retracting) {
			traversal.setPower(-1 * POWER);
		}
	}
	
	/** isFinished ************************************************************	
	 * Make this return true when this Command no longer needs to run execute() */
	public boolean isFinished() {
		if (!retracting) {
			return traversal.getWinchPosition() >= goalPos;
		}
		else {
			return traversal.getWinchPosition() <= 0;
		}
	}
	
	// Called once the command ends or is interrupted.
	public void end(boolean interrupted) {
		log.add("End", Log.Level.TRACE);
		traversal.stop();
	}
}