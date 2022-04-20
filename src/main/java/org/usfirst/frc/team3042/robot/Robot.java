package org.usfirst.frc.team3042.robot;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;

import org.usfirst.frc.team3042.lib.Log;
import org.usfirst.frc.team3042.robot.commands.ClimberTraversal_Toggle;
import org.usfirst.frc.team3042.robot.commands.autonomous.AutonomousMode_Default;
import org.usfirst.frc.team3042.robot.commands.autonomous.AutonomousMode_LeftTarmac;
import org.usfirst.frc.team3042.robot.commands.autonomous.AutonomousMode_RightTarmac;
import org.usfirst.frc.team3042.robot.commands.autonomous.helperCommands.PPMecanumControllerCommand;
import org.usfirst.frc.team3042.robot.subsystems.Climber;
import org.usfirst.frc.team3042.robot.subsystems.ClimberTraversal;
import org.usfirst.frc.team3042.robot.subsystems.Conveyor;
import org.usfirst.frc.team3042.robot.subsystems.Drivetrain;
import org.usfirst.frc.team3042.robot.subsystems.Intake;

import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.cameraserver.CameraServer;

/** Robot *********************************************************************
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the manifest file in the resource directory. */
public class Robot extends TimedRobot { 
	/** Configuration Constants ***********************************************/
	private static final Log.Level LOG_LEVEL = RobotMap.LOG_ROBOT;

	/** Create Subsystems *****************************************************/
	private Log log = new Log(LOG_LEVEL, "Robot");
	public static final Climber climber = new Climber();
	public static final ClimberTraversal traversal = new ClimberTraversal();
	public static final Conveyor conveyor  = new Conveyor();
	public static final Drivetrain drivetrain = new Drivetrain();
	public static final Intake intake = new Intake();
	public static final OI oi = new OI();;

	static ProfiledPIDController thetaController = new ProfiledPIDController(RobotMap.kP_THETA_CONTROLLER, 0, 0, drivetrain.getkThetaControllerConstraints());
	public static final PowerDistribution pdp = new PowerDistribution();
	
	CommandBase autonomousCommand;
	SendableChooser<CommandBase> chooser = new SendableChooser<CommandBase>();

	double goalAngle;
	UsbCamera camera1;
	int climberCurrentCount = 0;
	Timer currentTimer = new Timer();

	/** robotInit *************************************************************
	 * This function is run when the robot is first started up and should be used for any initialization code. */
	public void robotInit() {
		log.add("Robot Init", Log.Level.TRACE);

		thetaController.enableContinuousInput(-Math.PI, Math.PI);

		drivetrain.zeroGyro();
		drivetrain.resetEncoders();
		traversal.resetEncoder();
		
		// Autonomous Routines //
		chooser.setDefaultOption("Default Auto", new AutonomousMode_Default());
		chooser.addOption("2 ball (Left)", new AutonomousMode_LeftTarmac());
		chooser.addOption("3 ball (Right)", new AutonomousMode_RightTarmac());
		//chooser.addOption("4 Ball (Right)", new AutonomousMode_Ludicrous());

		//chooser.addOption("Straight TEST", constructTrajectoryCommand("Basic_Straight_Line_Path")); // This is only for tuning purposes
		//chooser.addOption("Curve TEST", constructTrajectoryCommand("Basic_Curve_Path")); // This is only for tuning purposes
				
		SmartDashboard.putData("Auto Mode", chooser);

		// Start up the webcam and configure its resolution and framerate
		camera1 = CameraServer.startAutomaticCapture(0);
		camera1.setResolution(320, 240);
		camera1.setFPS(15);
	}

	/** disabledInit **********************************************************
	 * This function is called once each time the robot enters Disabled mode.
	 * You can use it to reset any subsystem information you want to clear when the robot is disabled. */
	public void disabledInit() {
		log.add("Disabled Init", Log.Level.TRACE);
	}

	/** disabledPeriodic ******************************************************
	 * Called repeatedly while the robot is in disabled mode. */
	public void disabledPeriodic() {
		CommandScheduler.getInstance().run();
	}

	/** autonomousInit ********************************************************
	 * Runs once at the start of autonomous mode. */
	public void autonomousInit() {
		log.add("Autonomous Init", Log.Level.TRACE);

		drivetrain.resetEncoders();
		climber.retract();
		intake.retract();
		
		autonomousCommand = chooser.getSelected();
		
		// schedule the autonomous command
		if (autonomousCommand != null) {
			autonomousCommand.schedule();
		}
	}

	/** autonomousPeriodic ****************************************************
	 * This function is called periodically during autonomous */
	public void autonomousPeriodic() {
		CommandScheduler.getInstance().run();
		SmartDashboard.putNumber("Robot Speed", (Math.abs(drivetrain.getLeftFrontSpeed()) + Math.abs(drivetrain.getRightFrontSpeed()) + Math.abs(drivetrain.getLeftBackSpeed()) + Math.abs(drivetrain.getRightBackSpeed())) / 4.0); // Average drivetrain speed
		SmartDashboard.putNumber("Gyro Angle", drivetrain.getGyroAngle()); // The current gyroscope angle
		SmartDashboard.putNumber("Encoder Position (LF)", drivetrain.getLeftFrontPosition()); //The current right encoder position
		SmartDashboard.putNumber("Encoder Position (RF)", drivetrain.getRightFrontPosition()); //The current left encoder position
		SmartDashboard.putNumber("Encoder Position (LB)", drivetrain.getLeftBackPosition()); //The current right encoder position
		SmartDashboard.putNumber("Encoder Position (RB)", drivetrain.getRightBackPosition()); //The current left encoder position
	}
	
	/** teleopInit ************************************************************
	 * This function is called when first entering teleop mode. */
	public void teleopInit() {
		log.add("Teleop Init", Log.Level.TRACE);

		// This makes sure that the autonomous command stops running when teleop starts. 
		//If you want the autonomous command to continue until interrupted by another command, remove this line or comment it out.
		if (autonomousCommand != null) {
			autonomousCommand.cancel();
		}
		
		drivetrain.resetEncoders();
		climber.retract();

		goalAngle = drivetrain.getGyroAngle();
	}

	/** teleopPeriodic ********************************************************
	 * This function is called periodically during operator control */
	public void teleopPeriodic() {
		CommandScheduler.getInstance().run();
		SmartDashboard.putNumber("Robot Speed", (Math.abs(drivetrain.getLeftFrontSpeed()) + Math.abs(drivetrain.getRightFrontSpeed()) + Math.abs(drivetrain.getLeftBackSpeed()) + Math.abs(drivetrain.getRightBackSpeed())) / 4.0); // Average drivetrain speed
		SmartDashboard.putNumber("Gyro Angle", drivetrain.getGyroAngle()); // The current gyroscope angle
		SmartDashboard.putNumber("Encoder Position (LF)", drivetrain.getLeftFrontPosition()); //The current right encoder position
		SmartDashboard.putNumber("Encoder Position (RF)", drivetrain.getRightFrontPosition()); //The current left encoder position
		SmartDashboard.putNumber("Encoder Position (LB)", drivetrain.getLeftBackPosition()); //The current right encoder position
		SmartDashboard.putNumber("Encoder Position (RB)", drivetrain.getRightBackPosition()); //The current left encoder position
		SmartDashboard.putNumber("Traversal Winch Position", traversal.getWinchPosition()); // The current traversal winch position

		double ySpeed = oi.getYSpeed();
		double xSpeed = oi.getXSpeed();
		double zSpeed = oi.getZSpeed();
		
		// Displays the current of the left and right climber motors onto SmartDashBoard (shuffleboard)
		SmartDashboard.putNumber("Right Climber Current", pdp.getCurrent(14));		
		SmartDashboard.putNumber("Left Climber Current", pdp.getCurrent(2));
		SmartDashboard.putNumber("Climber Current Count", climberCurrentCount);	

		// Creates an instance of the ClimberTraversal_Toggle command
		ClimberTraversal_Toggle toggleTraversal = new ClimberTraversal_Toggle();

		// Checks whether the climbing arms' current is greater than x and if the traversal climber is already extended then it'll retact
		if(pdp.getCurrent(14) >= 15 && pdp.getCurrent(2) >= 15 && traversal.isRetracted() == false) {
			if(currentTimer.get() >= 0.1) {
				climberCurrentCount = 0;
			}
			climberCurrentCount++;
			currentTimer.stop();
			currentTimer.reset();
			currentTimer.start();
			if(climberCurrentCount >= 5) {
				toggleTraversal.schedule();
				climberCurrentCount = 0;
			}
		}

		if (Math.abs(zSpeed) > 0.01) { // If we are telling the robot to rotate, then let it rotate
			drivetrain.driveCartesian(ySpeed, xSpeed, zSpeed, drivetrain.getGyroAngle());
			goalAngle = drivetrain.getGyroAngle();
		}
		else { // Otherwise, use the gyro to maintain our current angle
			double error = goalAngle - drivetrain.getGyroAngle();
			
			double correction = RobotMap.kP_GYRO * error;

			correction = Math.min(RobotMap.MAX_POWER_GYRO, correction);
			correction = Math.max(-RobotMap.MAX_POWER_GYRO, correction);
			
			drivetrain.driveCartesian(ySpeed, xSpeed, -1 * correction, drivetrain.getGyroAngle());
		}
	} 

	public static SequentialCommandGroup constructTrajectoryCommand(String pathName, double velocityMax, double accelMax) { // Give this a path name and it will return a PPMecanumControllerCommand for that path :)
		
		PathPlannerTrajectory path = PathPlanner.loadPath(pathName, velocityMax, accelMax); 

		// Add kinematics to ensure max speed is actually obeyed
		PPMecanumControllerCommand mecanumControllerCommand = new PPMecanumControllerCommand(path, drivetrain::getPose, drivetrain.getkDriveKinematics(),

		// Position contollers
		new PIDController(RobotMap.kP_X_CONTROLLER, 0, 0),
		new PIDController(RobotMap.kP_Y_CONTROLLER, 0, 0),
		thetaController,

		drivetrain::setWheelSpeeds, drivetrain);

		return mecanumControllerCommand.andThen(() -> drivetrain.driveCartesian(0, 0, 0));
	}
}