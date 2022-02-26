package org.usfirst.frc.team3042.robot.commands;

import org.usfirst.frc.team3042.lib.Log;
import org.usfirst.frc.team3042.robot.Robot;
import org.usfirst.frc.team3042.robot.RobotMap;
import org.usfirst.frc.team3042.robot.subsystems.Drivetrain;

import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.wpilibj2.command.CommandBase;

/** Drivetrain Gyro Strafe **************************************************
 * Command for driving sideways using gyroscope feedback. */
public class Drivetrain_GyroStrafe extends CommandBase {
  
  	/** Configuration Constants ***********************************************/
	private static final Log.Level LOG_LEVEL = RobotMap.LOG_DRIVETRAIN;
	private static final double kP = RobotMap.kP_GYRO;
	private static final double CIRCUMFRENCE = RobotMap.WHEEL_DIAMETER * Math.PI;
	private static final double MAX_CORRECTION = RobotMap.MAX_POWER_GYRO;
	
	/** Instance Variables ****************************************************/
	Drivetrain drivetrain = Robot.drivetrain;
	Log log = new Log(LOG_LEVEL, SendableRegistry.getName(drivetrain));
	double strafePower, goalAngle, goalDistance;

  	/** Creates a new Drivetrain_GyroStrafe. */
  	public Drivetrain_GyroStrafe(double distance, double power) {
    log.add("Constructor", Log.Level.TRACE);
		strafePower = power;
		
		// convert distance to revolutions
		goalDistance = distance / CIRCUMFRENCE;

		addRequirements(drivetrain);
  	}

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    log.add("Initialize", Log.Level.TRACE);
		drivetrain.driveCartesian(0, 0, 0);
		goalAngle = drivetrain.getGyroAngle();
		drivetrain.resetEncoders();
  	}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double error = goalAngle - drivetrain.getGyroAngle();
		
		double correction = kP * error;

		correction = Math.min(MAX_CORRECTION, correction);
		correction = Math.max(-MAX_CORRECTION, correction);
		
		drivetrain.driveCartesian(strafePower, 0, -1 * correction);
 	 }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    boolean rightFrontGoalReached = Math.abs(drivetrain.getRightFrontPosition()) >= goalDistance;
		boolean rightBackGoalReached = Math.abs(drivetrain.getRightBackPosition()) >= goalDistance;
		return rightFrontGoalReached || rightBackGoalReached;
  	}

    // Called once the command ends or is interrupted.
	public void end(boolean interrupted) {
		log.add("End", Log.Level.TRACE);
		drivetrain.driveCartesian(0, 0, 0);
	}
}