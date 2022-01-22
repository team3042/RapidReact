package org.usfirst.frc.team3042.robot;

import org.usfirst.frc.team3042.lib.Log;
import org.usfirst.frc.team3042.robot.commands.autonomous.AutonomousMode_Default;
import org.usfirst.frc.team3042.robot.subsystems.Climber;
import org.usfirst.frc.team3042.robot.subsystems.Conveyor;
import org.usfirst.frc.team3042.robot.subsystems.Drivetrain;
import org.usfirst.frc.team3042.robot.subsystems.Intake;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.PowerDistribution;

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
	public static final Climber climber 			  = new Climber();
	public static final Conveyor conveyor 			  = new Conveyor();
	public static final Drivetrain drivetrain 			  = new Drivetrain();
	public static final Intake intake 			  		  = new Intake();
	public static final PowerDistribution pdp		  = new PowerDistribution();
	public static OI oi;
	
	Command autonomousCommand;
	SendableChooser<Command> chooser = new SendableChooser<Command>();

	UsbCamera camera1;

	/** robotInit *************************************************************
	 * This function is run when the robot is first started up and should be used for any initialization code. */
	public void robotInit() {
		log.add("Robot Init", Log.Level.TRACE);

		oi = new OI();

		drivetrain.zeroGyro();
		drivetrain.resetEncoders();
		
		// Autonomous Routines //
		chooser.setDefaultOption("Default Auto", new AutonomousMode_Default());
		//chooser.addOption("Option Name Here", new Command_Name());
				
		SmartDashboard.putData("Auto Mode", chooser);

		// Start up the webcam and configure its resolution and framerate
		camera1 = CameraServer.startAutomaticCapture(0);
		camera1.setResolution(640, 480);
		camera1.setFPS(15);
	}

	/** disabledInit **********************************************************
	 * This function is called once each time the robot enters Disabled mode.
	 * You can use it to reset any subsystem information you want to clear when the robot is disabled. */
	public void disabledInit() {
		log.add("Disabled Init", Log.Level.TRACE);
	}

	/** disabledPeriodic ******************************************************
	 * Called repeatedly while the robot is is disabled mode. */
	public void disabledPeriodic() {
		Scheduler.getInstance().run();
	}

	/** autonomousInit ********************************************************
	 * Runs once at the start of autonomous mode. */
	public void autonomousInit() {
		log.add("Autonomous Init", Log.Level.TRACE);

		drivetrain.zeroGyro();
		drivetrain.resetEncoders();
		
		autonomousCommand = chooser.getSelected();

		// schedule the autonomous command
		if (autonomousCommand != null) {
			autonomousCommand.start();
		}
	}

	/** autonomousPeriodic ****************************************************
	 * This function is called periodically during autonomous */
	public void autonomousPeriodic() {
		Scheduler.getInstance().run();
	}
	
	/** teleopInit ************************************************************
	 * This function is called when first entering teleop mode. */
	public void teleopInit() {
		log.add("Teleop Init", Log.Level.TRACE);
		
		drivetrain.zeroGyro();
		drivetrain.resetEncoders();
		
		// This makes sure that the autonomous command stops running when teleop starts. 
		//If you want the autonomous command to continue until interrupted by another command, remove this line or comment it out.
		if (autonomousCommand != null) {
			autonomousCommand.cancel();
		}
	}

	/** teleopPeriodic ********************************************************
	 * This function is called periodically during operator control */
	public void teleopPeriodic() {
		Scheduler.getInstance().run();
		SmartDashboard.putNumber("Robot Speed", (drivetrain.getLeftFrontSpeed() + drivetrain.getRightFrontSpeed() + drivetrain.getLeftBackSpeed() + drivetrain.getRightBackSpeed()) / 4.0); // Average speed of the left and right side
		SmartDashboard.putNumber("Gyro Angle", drivetrain.getGyroAngle()); // The current gyroscope angle

		SmartDashboard.putNumber("Encoder Position (LF)", drivetrain.getLeftFrontPosition()); //The current right encoder position
		SmartDashboard.putNumber("Encoder Position (RF)", drivetrain.getRightFrontPosition()); //The current left encoder position
		SmartDashboard.putNumber("Encoder Position (LB)", drivetrain.getLeftBackPosition()); //The current right encoder position
		SmartDashboard.putNumber("Encoder Position (RB)", drivetrain.getRightBackPosition()); //The current left encoder position
	} 
}