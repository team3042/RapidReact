package org.usfirst.frc.team3042.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SendableRegistry;

import org.usfirst.frc.team3042.lib.Log;
import org.usfirst.frc.team3042.robot.Robot;
import org.usfirst.frc.team3042.robot.RobotMap;
import org.usfirst.frc.team3042.robot.subsystems.Intake;

/** Intake *******************************************************************
 * Sets power to the intake */
public class Intake_Intake extends Command {
	/** Configuration Constants ***********************************************/
	private static final Log.Level LOG_LEVEL = RobotMap.LOG_INTAKE;
	private static final double POWER = RobotMap.INTAKE_POWER;
	
	/** Instance Variables ****************************************************/
	Intake intake = Robot.intake;
	Log log = new Log(LOG_LEVEL, SendableRegistry.getName(intake));
	int direction;

	/** Intake ****************************************************************
	 * Required subsystems will cancel commands when this command is run. */
	public Intake_Intake(int direction) {
		log.add("Constructor", Log.Level.TRACE);
		this.direction = direction;
		requires(intake);
	}

	/** initialize ************************************************************
	 * Called just before this Command runs the first time */
	protected void initialize() {
		log.add("Initialize", Log.Level.TRACE);

		intake.setPower(POWER * direction);
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
		intake.stop();
	}

	/** interrupted ***********************************************************
	 * Called when another command which requires one or more of the same
	 * subsystems is scheduled to run */
	protected void interrupted() {
		log.add("Interrupted", Log.Level.TRACE);
		intake.stop();
	}
}