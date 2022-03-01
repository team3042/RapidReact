package org.usfirst.frc.team3042.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.util.sendable.SendableRegistry;

import org.usfirst.frc.team3042.lib.Log;
import org.usfirst.frc.team3042.robot.Robot;
import org.usfirst.frc.team3042.robot.RobotMap;
import org.usfirst.frc.team3042.robot.subsystems.Intake;

/** Intake_Intake *******************************************************************
 * Sets power to the intake, either forwards or reverse */
public class Intake_Run extends CommandBase {
	/** Configuration Constants ***********************************************/
	private static final Log.Level LOG_LEVEL = RobotMap.LOG_INTAKE;
	private static final double POWER = RobotMap.INTAKE_POWER;
	
	/** Instance Variables ****************************************************/
	Intake intake = Robot.intake;
	Log log = new Log(LOG_LEVEL, SendableRegistry.getName(intake));
	int direction;

	/** Intake ****************************************************************
	 * Required subsystems will cancel commands when this command is run. */
	public Intake_Run(int direction) {
		log.add("Constructor", Log.Level.TRACE);
		this.direction = direction;
		addRequirements(intake);	
	}

	/** initialize ************************************************************
	 * Called just before this Command runs the first time */
	public void initialize() {
		log.add("Initialize", Log.Level.TRACE);
		intake.setPower(POWER * direction);
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
		intake.stop();
	}
}