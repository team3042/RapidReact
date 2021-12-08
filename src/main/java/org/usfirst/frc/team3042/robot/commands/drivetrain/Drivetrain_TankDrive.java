package org.usfirst.frc.team3042.robot.commands.drivetrain;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SendableRegistry;

import org.usfirst.frc.team3042.lib.Log;
import org.usfirst.frc.team3042.robot.OI;
import org.usfirst.frc.team3042.robot.Robot;
import org.usfirst.frc.team3042.robot.RobotMap;
import org.usfirst.frc.team3042.robot.subsystems.Drivetrain;

/** Drivetrain Tank Drive *****************************************************
 * Use joystick input to drive the robot. */
public class Drivetrain_TankDrive extends Command {
	/** Configuration Constants ***********************************************/
	private static final Log.Level LOG_LEVEL = RobotMap.LOG_DRIVETRAIN;
	private static final double ACCELERATION_MAX = RobotMap.ACCELERATION_MAX;
	
	/** Instance Variables ****************************************************/
	Drivetrain drivetrain = Robot.drivetrain;
	Log log = new Log(LOG_LEVEL, SendableRegistry.getName(drivetrain));
	OI oi = Robot.oi;
	double leftPowerOld, rightPowerOld;
	Timer timer = new Timer();
	
	/** Drivetrain Tank Drive *************************************************
	 * Required subsystems will cancel commands when this command is run. */
	public Drivetrain_TankDrive() {
		log.add("Constructor", Log.Level.TRACE);
		requires(drivetrain);
	}

	/** initialize ************************************************************
	 * Called just before this Command runs the first time */
	protected void initialize() {
		log.add("Initialize", Log.Level.TRACE);
				
		drivetrain.stop();
		leftPowerOld = 0.0;
		rightPowerOld = 0.0;
		
		timer.start();
		timer.reset();
	}

	/** execute ***************************************************************
	 * Called repeatedly when this Command is scheduled to run */
	protected void execute() {
		double leftPower = oi.getDriveLeft();
		double rightPower = oi.getDriveRight();
		
		double dt = timer.get();
		timer.reset();
		leftPower = restrictAcceleration(leftPower, leftPowerOld, dt);
		rightPower = restrictAcceleration(rightPower, rightPowerOld, dt);	
		
		drivetrain.setPower(leftPower, rightPower);
		
		leftPowerOld = leftPower;
		rightPowerOld = rightPower;
	}
	
	/** restrictAcceleration **************************************************/
	private double restrictAcceleration(double goalPower, 
		double currentPower, double dt) {
		double maxDeltaPower = ACCELERATION_MAX * dt;
		double deltaPower = Math.abs(goalPower - currentPower);
		double deltaSign = (goalPower < currentPower) ? -1.0 : 1.0;
		
		deltaPower = Math.min(maxDeltaPower, deltaPower);
		goalPower = currentPower + deltaSign * deltaPower;

		return goalPower;
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