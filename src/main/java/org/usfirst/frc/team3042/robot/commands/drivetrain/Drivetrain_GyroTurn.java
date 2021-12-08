package org.usfirst.frc.team3042.robot.commands.drivetrain;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SendableRegistry;

import org.usfirst.frc.team3042.lib.Log;
import org.usfirst.frc.team3042.robot.Robot;
import org.usfirst.frc.team3042.robot.RobotMap;
import org.usfirst.frc.team3042.robot.subsystems.Drivetrain;

/** Drivetrain Gyro Turn ******************************************************
 * Command for turning in place to a set angle. */
public class Drivetrain_GyroTurn extends Command {
	/** Configuration Constants ***********************************************/
	private static final Log.Level LOG_LEVEL = RobotMap.LOG_DRIVETRAIN;
	private static final double kP = RobotMap.kP_GYRO;
	private static final double kI = RobotMap.kI_GYRO;
	private static final double kD = RobotMap.kD_GYRO;
	private static final double ANGLE_TOLERANCE = RobotMap.ANGLE_TOLERANCE;
	private static final double MAX_POWER = RobotMap.MAX_POWER_GYRO;
	
	/** Instance Variables ****************************************************/
	Drivetrain drivetrain = Robot.drivetrain;
	Log log = new Log(LOG_LEVEL, SendableRegistry.getName(drivetrain));
	double lastError, integralError, goalAngle;
	
	/** Drivetrain Gyro Turn ************************************************** 
	 * Required subsystems will cancel commands when this command is run.
	 * distance is given in physical units matching the wheel diameter unit
	 * speed is given in physical units per second. The physical units should 
	 * match that of the Wheel diameter.
	 * @param angle (degrees) */
	public Drivetrain_GyroTurn(double angle) {
		log.add("Constructor", Log.Level.TRACE);
		requires(drivetrain);
		goalAngle = angle;
	}
	
	/** initialize ************************************************************
	 * Called just before this Command runs the first time */
	protected void initialize() {
		log.add("Initialize", Log.Level.TRACE);
		drivetrain.stop();
		lastError = 0.0;
		integralError = 0.0;
		drivetrain.zeroGyro();
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
		
		correction = Math.min(MAX_POWER, correction);
		correction = Math.max(-MAX_POWER, correction);
	
		drivetrain.setPower(correction, -correction);
		
		log.add("***** " + correction, Log.Level.DEBUG);

		lastError = error;
	}
	
	/** isFinished ************************************************************	
	 * Make this return true when this Command no longer needs to run execute() */
	protected boolean isFinished() {
		return Math.abs(lastError) < ANGLE_TOLERANCE;
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