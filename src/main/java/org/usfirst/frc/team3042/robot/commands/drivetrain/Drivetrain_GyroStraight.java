package org.usfirst.frc.team3042.robot.commands.drivetrain;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.util.sendable.SendableRegistry;

import org.usfirst.frc.team3042.lib.Log;
import org.usfirst.frc.team3042.robot.Robot;
import org.usfirst.frc.team3042.robot.RobotMap;
import org.usfirst.frc.team3042.robot.subsystems.Drivetrain;

/** Drivetrain Gyro Straight **************************************************
 * Command for driving straight using gyroscope feedback. */
public class Drivetrain_GyroStraight extends Command {
	/** Configuration Constants ***********************************************/
	private static final Log.Level LOG_LEVEL = RobotMap.LOG_DRIVETRAIN;
	private static final double kP = RobotMap.kP_GYRO;
	private static final double kI = RobotMap.kI_GYRO;
	private static final double kD = RobotMap.kD_GYRO;
	private static final double kF_LEFT = RobotMap.kF_DRIVE_LEFT;
	private static final double kF_RIGHT = RobotMap.kF_DRIVE_RIGHT;
	private static final double CIRCUMFRENCE = RobotMap.WHEEL_DIAMETER * Math.PI;
	private static final double MAX_CORRECTION = RobotMap.MAX_POWER_GYRO;
	
	/** Instance Variables ****************************************************/
	Drivetrain drivetrain = Robot.drivetrain;
	Log log = new Log(LOG_LEVEL, SendableRegistry.getName(drivetrain));
	double leftPower, rightPower, lastError, integralError;
	double goalAngle, goalDistance;
	
	/** Drivetrain Gyro Straight **********************************************
	 * Required subsystems will cancel commands when this command is run.
	 * distance is given in physical units matching the wheel diameter unit
	 * speed is given in physical units per second. The physical units should 
	 * match that of the Wheel diameter. */
	public Drivetrain_GyroStraight(double distance, double speed) {
		log.add("Constructor", Log.Level.TRACE);
		requires(drivetrain);
		
		// convert distance to revolutions
		goalDistance = distance / CIRCUMFRENCE;
		
		// Find the power level for the given speed
		double rpm = speed * 60.0 / CIRCUMFRENCE;		
		leftPower = drivetrain.rpmToPower(rpm, kF_LEFT);
		rightPower = drivetrain.rpmToPower(rpm, kF_RIGHT);
	}
	
	/** initialize ************************************************************
	 * Called just before this Command runs the first time */
	protected void initialize() {
		log.add("Initialize", Log.Level.TRACE);

		drivetrain.stop();
		goalAngle = drivetrain.getAngle();
		lastError = 0.0;
		integralError = 0.0;
		drivetrain.resetEncoders();
	}

	/** execute ***************************************************************
	 * Called repeatedly when this Command is scheduled to run */
	protected void execute() {
		double error = goalAngle - drivetrain.getAngle();
		integralError += error;
		double deltaError = error - lastError;
		
		double Pterm = kP * error;
		double Iterm = kI * integralError;
		double Dterm = kD * deltaError;
		
		double correction = Pterm + Iterm + Dterm;
		correction = Math.min(MAX_CORRECTION, correction);
		correction = Math.max(-MAX_CORRECTION, correction);
		
		drivetrain.setPower(leftPower - correction, rightPower + correction);
		
		lastError = error;
	}
	
	/** isFinished ************************************************************	
	 * Make this return true when this Command no longer needs to run execute() */
	protected boolean isFinished() {
		boolean leftGoalReached = Math.abs(drivetrain.getLeftPosition()) >= goalDistance;
		boolean rightGoalReached = Math.abs(drivetrain.getRightPosition()) >= goalDistance;
		return leftGoalReached || rightGoalReached;
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