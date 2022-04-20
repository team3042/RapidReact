package org.usfirst.frc.team3042.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.util.sendable.SendableRegistry;

import org.usfirst.frc.team3042.lib.Log;
import org.usfirst.frc.team3042.robot.Robot;
import org.usfirst.frc.team3042.robot.RobotMap;
import org.usfirst.frc.team3042.robot.subsystems.ClimberTraversal;

/** ClimberTraversal_Manual *******************************************************************
 * Manually operate the traversal climbing hooks */
public class ClimberTraversal_Manual extends CommandBase {
	/** Configuration Constants ***********************************************/
	private static final Log.Level LOG_LEVEL = RobotMap.LOG_CLIMBER_TRAVERSAL;
	private static double POWER = RobotMap.TRAVERSAL_CLIMBER_POWER;

	/** Instance Variables ****************************************************/
	ClimberTraversal traversal = Robot.traversal;
	Log log = new Log(LOG_LEVEL, SendableRegistry.getName(traversal));
	int direction;

	/** ClimberTraversal_Manual ****************************************************************
	 * Required subsystems will cancel commands when this command is run. */
	public ClimberTraversal_Manual(int Direction) {
		log.add("Constructor", Log.Level.TRACE);
		addRequirements(traversal);
		direction = Direction;
	}

	/** initialize ************************************************************
	 * Called just before this Command runs the first time */
	public void initialize() {
		traversal.setPower(POWER * direction);
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
		traversal.stop();
	}
}