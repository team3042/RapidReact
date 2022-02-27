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
	//private static final int DIO_LIMITSWITCH_RIGHT = RobotMap.DIO_LIMITSWITCH_CLIMBER_RIGHT;
	//private static final int DIO_LIMITSWITCH_LEFT = RobotMap.DIO_LIMITSWITCH_CLIMBER_LEFT;
	
	/** Instance Variables ****************************************************/
  	Climber climber = Robot.climber;
	Log log = new Log(LOG_LEVEL, SendableRegistry.getName(climber));
	//DigitalInput limitRight = new DigitalInput(DIO_LIMITSWITCH_RIGHT);
	//DigitalInput limitLeft = new DigitalInput(DIO_LIMITSWITCH_LEFT);
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
	public void execute() {
		/*if (direction == -1) { // Make sure climber does not go down past limit switches
			if(limitLeft.get() && !limitRight.get()) {
				climber.setLeftPower(0);
				climber.setRightPower(POWER * direction);
			}
			else if(!limitLeft.get() && limitRight.get()) {
				climber.setLeftPower(POWER * direction);
				climber.setRightPower(0);
			}
			else if(limitLeft.get() && limitRight.get()) {
				climber.setPower(0);
			}
			else {
				climber.setPower(POWER * direction);
			}
		}
		else {
			climber.setPower(POWER * direction);
		}*/
	}
	
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