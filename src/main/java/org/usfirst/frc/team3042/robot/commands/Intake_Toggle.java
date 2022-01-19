package org.usfirst.frc.team3042.robot.commands;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.util.sendable.SendableRegistry;

import org.usfirst.frc.team3042.lib.Log;
import org.usfirst.frc.team3042.robot.Robot;
import org.usfirst.frc.team3042.robot.RobotMap;
import org.usfirst.frc.team3042.robot.subsystems.Intake;

/** Intake_Toggle *******************************************************************
 * Extends and retracts the intake */
public class Intake_Toggle extends Command {
	/** Configuration Constants ***********************************************/
	private static final Log.Level LOG_LEVEL = RobotMap.LOG_INTAKE;
	private static final int RIGHT_ID = RobotMap.RIGHT_INTAKE_SOLENOID;
	private static final int LEFT_ID = RobotMap.LEFT_INTAKE_SOLENOID;
	
	/** Instance Variables ****************************************************/
	Intake intake = Robot.intake;
	Log log = new Log(LOG_LEVEL, SendableRegistry.getName(intake));
	Solenoid rightIntakeSolenoid = new Solenoid(PneumaticsModuleType.CTREPCM, RIGHT_ID);
	Solenoid leftIntakeSolenoid = new Solenoid(PneumaticsModuleType.CTREPCM, LEFT_ID);
	boolean isRetracted = true;

	/** Intake ****************************************************************
	 * Required subsystems will cancel commands when this command is run. */
	public Intake_Toggle(int direction) {
		log.add("Constructor", Log.Level.TRACE);
		requires(intake);
	}
    public void toggle(){
    	if (isRetracted == true){
    		extend();
    	}
    	else {
    		retract();
    	}
    }
	public void extend() {
		rightIntakeSolenoid.set(true);
		leftIntakeSolenoid.set(true);
		isRetracted = false;
	}
	public void retract() {
		rightIntakeSolenoid.set(false);
		leftIntakeSolenoid.set(false);
		isRetracted = true;
	}

	/** initialize ************************************************************
	 * Called just before this Command runs the first time */
	protected void initialize() {
		log.add("Initialize", Log.Level.TRACE);
		toggle();
	}

	/** execute ***************************************************************
	 * Called repeatedly when this Command is scheduled to run */
	protected void execute() {}
	
	/** isFinished ************************************************************	
	 * Make this return true when this Command no longer needs to run execute() */
	protected boolean isFinished() {
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