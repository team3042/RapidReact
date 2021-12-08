package org.usfirst.frc.team3042.robot;

import java.io.IOException;
import java.nio.file.Path;

import org.usfirst.frc.team3042.lib.Log;
import org.usfirst.frc.team3042.robot.commands.autonomous.AutonomousMode_Default;
import org.usfirst.frc.team3042.robot.subsystems.Drivetrain;
import org.usfirst.frc.team3042.robot.subsystems.Intake;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryUtil;
import edu.wpi.cscore.UsbCamera;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.PowerDistributionPanel;

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
	public static final Drivetrain drivetrain 			  = new Drivetrain();
	public static final Intake intake 			  		  = new Intake();
	public static final PowerDistributionPanel pdp		  = new PowerDistributionPanel();
	public static OI oi;
	
	Command autonomousCommand;
	SendableChooser<Command> chooser = new SendableChooser<Command>();

	UsbCamera camera1;

	public String color;
	boolean ColorRecieved = false;

	/** robotInit *************************************************************
	 * This function is run when the robot is first started up and should be used for any initialization code. */
	public void robotInit() {
		log.add("Robot Init", Log.Level.TRACE);

		oi = new OI();

		drivetrain.zeroGyro();
		drivetrain.getEncoders().reset();
		
		// Infinite Recharge Autonomous Routines
		chooser.setDefaultOption("Default Auto", new AutonomousMode_Default());
		//chooser.addOption("Trench Six Balls", new AutonomousMode_Trench());
		//chooser.addOption("Delayed Shoot", new AutonomousMode_Delayed());
				
		SmartDashboard.putData("Auto Mode", chooser);

		// Start up the webcam and configure its resolution and framerate
		camera1 = CameraServer.getInstance().startAutomaticCapture(0);
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
	 * Called repeatedly while the robot is is disabled mode. */
	public void disabledPeriodic() {
		Scheduler.getInstance().run();
	}

	/** autonomousInit ********************************************************
	 * Runs once at the start of autonomous mode. */
	public void autonomousInit() {
		log.add("Autonomous Init", Log.Level.TRACE);
		ColorRecieved = false;
		SmartDashboard.putString("Color:", "Capacity Not Reached");

		drivetrain.zeroGyro();
		drivetrain.getEncoders().reset();
		
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
		ColorRecieved = false;
		
		drivetrain.zeroGyro();
		drivetrain.getEncoders().reset();
		
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
		
		SmartDashboard.putNumber("Drivetrain Speed", (drivetrain.getEncoders().getLeftSpeed() + drivetrain.getEncoders().getRightSpeed()) / 2.0); // Average speed of the left and right side

		//Read the assigned control panel color from the FMS and display it on the dashboard
		color = DriverStation.getInstance().getGameSpecificMessage();
		if (color.length() > 0) {
			ColorRecieved = true;
			switch (color.charAt(0)) {
				case 'B' :
				SmartDashboard.putString("Color:", "Blue");
				break;
				case 'G' :
				SmartDashboard.putString("Color:", "Green");
				break;
				case 'R' :
				SmartDashboard.putString("Color:", "Red");
				break;
				case 'Y' :
				SmartDashboard.putString("Color:", "Yellow");
				break;
				default :
				SmartDashboard.putString("Color:", "ERROR");
				break;
			}
		}
		else if (!ColorRecieved) {
			SmartDashboard.putString("Color:", "Capacity Not Reached");
		}
	} 

	// Takes the file location of a PathWeaver json file and builds it into a drivable trajectory
	public static Trajectory buildTrajectory(String trajectoryJSON) {
		
   		// Generate a trajectory to follow. All units should be in meters!
    	Trajectory trajectory = new Trajectory();
		
		try {
  			Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(trajectoryJSON);
  			trajectory = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
		} catch (IOException ex) {
  			DriverStation.reportError("Unable to access file: " + trajectoryJSON, ex.getStackTrace());
		}
		
		return trajectory;
	}
}