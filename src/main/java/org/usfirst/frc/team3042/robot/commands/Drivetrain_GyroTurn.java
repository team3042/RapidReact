package org.usfirst.frc.team3042.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.util.sendable.SendableRegistry;

import org.usfirst.frc.team3042.lib.Log;
import org.usfirst.frc.team3042.robot.Robot;
import org.usfirst.frc.team3042.robot.RobotMap;
import org.usfirst.frc.team3042.robot.subsystems.Drivetrain;

/** Drivetrain Gyro Turn ******************************************************
 * Command for turning in place to a set angle. */
public class Drivetrain_GyroTurn extends CommandBase {
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
		goalAngle = angle;
		addRequirements(drivetrain);
	}
	
	/** initialize ************************************************************
	 * Called just before this Command runs the first time */
	public void initialize() {
		log.add("Initialize", Log.Level.TRACE);
		drivetrain.driveCartesian(0, 0, 0);
		lastError = 0.0;
		integralError = 0.0;
		drivetrain.zeroGyro();
	}

	/** execute ***************************************************************
	 * Called repeatedly when this Command is scheduled to run */
	public void execute() {
		double error = goalAngle - drivetrain.getGyroAngle();
		integralError += error;
		double deltaError = error - lastError;
		
		double Pterm = kP * error;
		double Iterm = kI * integralError;
		double Dterm = kD * deltaError;
		
		double correction = Pterm + Iterm + Dterm;
		
		correction = Math.min(MAX_POWER, correction);
		correction = Math.max(-MAX_POWER, correction);
	
		drivetrain.driveCartesian(0, 0, -1 * correction);		
		
		log.add("***** " + correction, Log.Level.DEBUG);

		lastError = error;
	}
	
	/** isFinished ************************************************************	
	 * Make this return true when this Command no longer needs to run execute() */
	public boolean isFinished() {
		return Math.abs(lastError) < ANGLE_TOLERANCE;
	}

	// Called once the command ends or is interrupted.
	public void end(boolean interrupted) {
		log.add("End", Log.Level.TRACE);
		drivetrain.driveCartesian(0, 0, 0);
	}
}