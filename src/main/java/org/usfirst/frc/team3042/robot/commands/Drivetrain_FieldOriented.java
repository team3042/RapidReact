package org.usfirst.frc.team3042.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.util.sendable.SendableRegistry;

import org.usfirst.frc.team3042.lib.Log;
import org.usfirst.frc.team3042.robot.OI;
import org.usfirst.frc.team3042.robot.Robot;
import org.usfirst.frc.team3042.robot.RobotMap;
import org.usfirst.frc.team3042.robot.subsystems.Drivetrain;

/** Drivetrain Field Oriented *****************************************************
 * Use joystick input to manually drive the robot at a constant angle in relation to the field */
public class Drivetrain_FieldOriented extends CommandBase {
	/** Configuration Constants ***********************************************/
	private static final Log.Level LOG_LEVEL = RobotMap.LOG_DRIVETRAIN;
	private static final double ACCELERATION_MAX = RobotMap.ACCELERATION_MAX;
	
	/** Instance Variables ****************************************************/
	Drivetrain drivetrain = Robot.drivetrain;
	Log log = new Log(LOG_LEVEL, SendableRegistry.getName(drivetrain));
	OI oi = Robot.oi;
	double xSpeedOld, ySpeedOld, zSpeedOld;
	Timer timer = new Timer();
	
	/** Drivetrain Field Oriented *************************************************
	 * Required subsystems will cancel commands when this command is run. */
	public Drivetrain_FieldOriented() {
		log.add("Constructor", Log.Level.TRACE);
		addRequirements(drivetrain);
	}

	/** initialize ************************************************************
	 * Called just before this Command runs the first time */
	public void initialize() {
		log.add("Initialize", Log.Level.TRACE);
				
		drivetrain.stop();
		xSpeedOld = 0.0;
		ySpeedOld = 0.0;
		zSpeedOld = 0.0;
		
		timer.start();
		timer.reset();
	}

	/** execute ***************************************************************
	 * Called repeatedly when this Command is scheduled to run */
	public void execute() {
		double ySpeed = oi.getYSpeed();
		double xSpeed = oi.getXSpeed();
		double zSpeed = oi.getZSpeed();
		
		double dt = timer.get();
		timer.reset();

		ySpeed = restrictAcceleration(ySpeed, ySpeedOld, dt);
		xSpeed = restrictAcceleration(xSpeed, xSpeedOld, dt);
		zSpeed = restrictAcceleration(zSpeed, zSpeedOld, dt);
		
		drivetrain.driveCartesian(ySpeed, xSpeed, zSpeed, drivetrain.getGyroAngle());
		
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
	public boolean isFinished() {
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