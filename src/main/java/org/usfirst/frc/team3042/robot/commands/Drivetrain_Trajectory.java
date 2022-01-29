package org.usfirst.frc.team3042.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.MecanumControllerCommand;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.util.sendable.SendableRegistry;

import com.pathplanner.lib.PathPlanner;

import org.usfirst.frc.team3042.lib.Log;
import org.usfirst.frc.team3042.robot.Robot;
import org.usfirst.frc.team3042.robot.RobotMap;
import org.usfirst.frc.team3042.robot.subsystems.Drivetrain;

public class Drivetrain_Trajectory extends CommandBase {
	/** Configuration Constants ***********************************************/
	private static final Log.Level LOG_LEVEL = RobotMap.LOG_DRIVETRAIN;
	private static final double maxAcceleration = RobotMap.ACCELERATION_MAX_MPS;
 	private static final double maxVelocity = RobotMap.VELOCITY_MAX_MPS;
  	private static final double kPFrontLeftVel = RobotMap.kP_FRONT_LEFT_VELOCITY;
  	private static final double kPRearLeftVel = RobotMap.kP_BACK_LEFT_VELOCITY;
 	private static final double kPFrontRightVel = RobotMap.kP_FRONT_RIGHT_VELOCITY;
 	private static final double kPRearRightVel = RobotMap.kP_BACK_RIGHT_VELOCITY;
  	private static final double kPXController = RobotMap.kP_X_CONTROLLER;
  	private static final double kPYController = RobotMap.kP_Y_CONTROLLER;
  	private static final double kPThetaController = RobotMap.kP_THETA_CONTROLLER;
	private static final double kMaxAngularSpeedRadiansPerSecond = RobotMap.kMAX_ANGULAR_SPEED_RADIANS_PER_SECOND;
	private static final double kMaxAngularSpeedRadiansPerSecondSquared = RobotMap.kMAX_ANGULAR_SPEED_RADIANS_PER_SECOND_SQUARED;

 	 private static final TrapezoidProfile.Constraints kThetaControllerConstraints =
                    new TrapezoidProfile.Constraints(kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);

	/** Instance Variables ****************************************************/
	Drivetrain drivetrain = Robot.drivetrain;
	Log log = new Log(LOG_LEVEL, SendableRegistry.getName(drivetrain));
  	Trajectory path;

	/** Drivetrain Trajectory *************************************************
	 * Required subsystems will cancel commands when this command is run. */
	public Drivetrain_Trajectory(String pathName) {
		log.add("Constructor", Log.Level.TRACE);
    	path = PathPlanner.loadPath(pathName, maxVelocity, maxAcceleration); 
    	addRequirements(drivetrain);
	}

	/** initialize ************************************************************
	 * Called just before this Command runs the first time */
	public void initialize() {
		log.add("Initialize", Log.Level.TRACE);

    // Add kinematics to ensure max speed is actually obeyed
    MecanumControllerCommand mecanumControllerCommand = new MecanumControllerCommand(
      path,
      drivetrain::getPose,
      drivetrain.getkFeedforward(),
      drivetrain.getkDriveKinematics(),

      // Position contollers
      new PIDController(kPXController, 0, 0),
      new PIDController(kPYController, 0, 0),
      new ProfiledPIDController(kPThetaController, 0, 0, kThetaControllerConstraints),

      // Needed for normalizing wheel speeds
      maxVelocity,

      // Velocity PIDs
      new PIDController(kPFrontLeftVel, 0, 0),
      new PIDController(kPRearLeftVel, 0, 0),
      new PIDController(kPFrontRightVel, 0, 0),
      new PIDController(kPRearRightVel, 0, 0),
      drivetrain::getCurrentWheelSpeeds,
      drivetrain::setDriveMotorControllersVolts, // Consumer for the output motor voltages
      drivetrain);

      drivetrain.resetOdometry(path.getInitialPose());

      mecanumControllerCommand.andThen(() -> drivetrain.stop());
	}

	/** execute ***************************************************************
	 * Called repeatedly when this Command is scheduled to run */	
	public void execute() {
    	drivetrain.updateOdometry();
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