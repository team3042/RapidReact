package org.usfirst.frc.team3042.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.util.sendable.SendableRegistry;

import org.usfirst.frc.team3042.lib.Log;
import org.usfirst.frc.team3042.robot.OI;
import org.usfirst.frc.team3042.robot.Robot;
import org.usfirst.frc.team3042.robot.RobotMap;
import org.usfirst.frc.team3042.robot.subsystems.Drivetrain;

/** Drivetrain Mecanum Drive *****************************************************
 * Use joystick input to manually drive the robot */
public class Drivetrain_MecanumDrive extends Command {
	/** Configuration Constants ***********************************************/
	private static final Log.Level LOG_LEVEL = RobotMap.LOG_DRIVETRAIN;
	private static final double ACCELERATION_MAX = RobotMap.ACCELERATION_MAX;
	
	/** Instance Variables ****************************************************/
	Drivetrain drivetrain = Robot.drivetrain;
	Log log = new Log(LOG_LEVEL, SendableRegistry.getName(drivetrain));
	OI oi = Robot.oi;
	double ySpeedOld, xSpeedOld, zSpeedOld;
	Timer timer = new Timer();
	
	/** Drivetrain Mecanum Drive *************************************************
	 * Required subsystems will cancel commands when this command is run. */
	public Drivetrain_MecanumDrive() {
		log.add("Constructor", Log.Level.TRACE);
		requires(drivetrain);
	}

	/** initialize ************************************************************
	 * Called just before this Command runs the first time */
	protected void initialize() {
		log.add("Initialize", Log.Level.TRACE);
				
		drivetrain.stop();
		ySpeedOld = 0.0;
		xSpeedOld = 0.0;
		zSpeedOld = 0.0;
		
		timer.start();
		timer.reset();
	}

	/** execute ***************************************************************
	 * Called repeatedly when this Command is scheduled to run */	
	protected void execute() {
		double ySpeed = oi.getXSpeed();
		double xSpeed = oi.getYSpeed();
		double zSpeed = oi.getZSpeed();
		
		double dt = timer.get();
		timer.reset();

		ySpeed = restrictAcceleration(ySpeed, ySpeedOld, dt);
		xSpeed = restrictAcceleration(xSpeed, xSpeedOld, dt);
		zSpeed = restrictAcceleration(zSpeed, zSpeedOld, dt);
		
		drivetrain.driveCartesian(ySpeed, xSpeed, zSpeed);
		
		ySpeedOld = ySpeed;
		xSpeedOld = xSpeed;
		zSpeedOld = zSpeed;
	}
	
	/** restrictAcceleration **************************************************/
	private double restrictAcceleration(double goalSpeed, double currentSpeed, double dt) {
		double maxDeltaSpeed = ACCELERATION_MAX * dt;
		double deltaSpeed = Math.abs(goalSpeed - currentSpeed);
		double deltaSign = (goalSpeed < currentSpeed) ? -1.0 : 1.0;
		
		deltaSpeed = Math.min(maxDeltaSpeed, deltaSpeed);
		goalSpeed = currentSpeed + deltaSign * deltaSpeed;

		return goalSpeed;
	}
	
	/** isFinished ************************************************************	
	 * Make this return true when this Command no longer needs to run execute() */
	protected boolean isFinished() {
		return false;
	}

	/** end *******************************************************************
	 * Called once after isFinished returns true */
	protected void end() {
		log.add("End", Log.Level.TRACE);
		drivetrain.stop();
	}

	/** interrupted ***********************************************************
	 * Called when another command which requires one or more of the same
	 * subsystems is scheduled to run */
	protected void interrupted() {
		log.add("Interrupted", Log.Level.TRACE);
		drivetrain.stop();
	}
}