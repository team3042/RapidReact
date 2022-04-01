package org.usfirst.frc.team3042.robot;

import org.usfirst.frc.team3042.lib.Log;
import org.usfirst.frc.team3042.robot.commands.ClimberTraversal_Manual;
import org.usfirst.frc.team3042.robot.commands.ClimberTraversal_Toggle;
import org.usfirst.frc.team3042.robot.commands.Climber_Run;
import org.usfirst.frc.team3042.robot.commands.Conveyor_Advance;
import org.usfirst.frc.team3042.robot.commands.Conveyor_Run;
import org.usfirst.frc.team3042.robot.commands.Intake_Run;
import org.usfirst.frc.team3042.robot.subsystems.Climber;
import org.usfirst.frc.team3042.robot.subsystems.Drivetrain;
import org.usfirst.frc.team3042.robot.subsystems.Intake;

import edu.wpi.first.wpilibj2.command.InstantCommand;

/** OI ************************************************************************
 * This class is the glue that binds the controls on the physical operator
 * interface to the commands and command groups that allow control of the robot. */
public class OI {	
	/** Configuration Constants ***********************************************/
	private static final int USB_GAMEPAD = RobotMap.USB_GAMEPAD;
	private static final int USB_JOY_LEFT = RobotMap.USB_JOYSTICK_LEFT;
	private static final int USB_JOY_RIGHT = RobotMap.USB_JOYSTICK_RIGHT;
	private static final int JOYSTICK_X_AXIS = Gamepad.JOY_X_AXIS;
	private static final int JOYSTICK_Y_AXIS = Gamepad.JOY_Y_AXIS;
	private static final int JOYSTICK_Z_AXIS = Gamepad.JOY_Z_AXIS;
	private static final double JOYSTICK_DRIVE_SCALE = RobotMap.JOYSTICK_DRIVE_SCALE;
	private static final double JOYSTICK_DRIVE_SCALE_LOW = RobotMap.JOYSTICK_DRIVE_SCALE_LOW;
	
	/** Instance Variables ****************************************************/
	Log log = new Log(RobotMap.LOG_OI, "OI");
	public static Gamepad gamepad, joyLeft, joyRight;
	public static double CURRENT_DRIVE_SCALE = JOYSTICK_DRIVE_SCALE;
	public static boolean isLowScale = false;

	Drivetrain drivetrain = Robot.drivetrain;
	Climber climber = Robot.climber;
	Intake intake = Robot.intake;

	int driveAxisX, driveAxisY, driveAxisZ;

	/** OI ********************************************************************
	 * Assign commands to the buttons and triggers*/
	public OI() {
		log.add("OI Constructor", Log.Level.TRACE);
		
		gamepad = new Gamepad(USB_GAMEPAD);
		
		// Setup Driving Controls //
		joyLeft = new Gamepad(USB_JOY_LEFT);
		joyRight = new Gamepad(USB_JOY_RIGHT);
		driveAxisX = JOYSTICK_X_AXIS;
		driveAxisY = JOYSTICK_Y_AXIS;
		driveAxisZ = JOYSTICK_Z_AXIS;

		joyLeft.button1.whenPressed(new InstantCommand(drivetrain::zeroGyro, drivetrain)); // Zero the gyro, this is helpful for field-oriented driving
		joyRight.button1.whenPressed(new InstantCommand(this::toggleScale)); // Toggle into slow driving mode
		joyRight.button1.whenReleased(new InstantCommand(this::toggleScale)); // Toggle out of slow driving mode
		
		// Intake Controls //
		gamepad.LB.whileHeld(new Intake_Run(1)); // run the intake
		gamepad.LT.whileActiveOnce(new Intake_Run(-1)); // reverse the intake

		gamepad.A.whenPressed(new InstantCommand(intake::toggle, intake)); // extend or retract the intake

		// Climber Controls //
		gamepad.POVUp.whileActiveOnce(new Climber_Run(1)); // Raise the climber
		gamepad.POVDown.whileActiveOnce(new Climber_Run(-1)); // Lower the climber

		gamepad.RightJoyUp.whileActiveOnce(new ClimberTraversal_Manual(1)); // Extend the traversal hooks outward
		gamepad.RightJoyDown.whileActiveOnce(new ClimberTraversal_Manual(-1)); // Retract the traversal hooks inward

		gamepad.X.whenPressed(new ClimberTraversal_Toggle()); // Extend or retract the traversal hooks
		
		// Conveyor Controls //
		gamepad.RB.whileHeld(new Conveyor_Run(1)); // run the converyor
		gamepad.RT.whenActive(new Conveyor_Advance()); // Auto advance the conveyor

		gamepad.B.whileHeld(new Conveyor_Run(-0.5)); // reverse the converyor
	}
	
	/** Access to the driving axes values *****************************
	 * A negative can be added to make pushing forward positive/negative. */
	public double getXSpeed() {
		double joystickValue = joyRight.getRawAxis(driveAxisY);
		joystickValue = scaleJoystick(joystickValue);
		return -1 * joystickValue; // Multiply by -1 to reverse direction
	}
	public double getYSpeed() {
		double joystickValue = joyRight.getRawAxis(driveAxisX);
		joystickValue = scaleJoystick(joystickValue);
		return joystickValue; 
	}
	public double getZSpeed() {
		double joystickValue = joyLeft.getRawAxis(driveAxisX);
		joystickValue = scaleJoystick(joystickValue);
		return 0.6 * joystickValue; // Scale turning to be 60% the speed of driving for better control
	}	
	private double scaleJoystick(double joystickValue) {
		joystickValue *= CURRENT_DRIVE_SCALE;
		return joystickValue;
	}

	/** Methods for scaling drivetrain speeds *******************************************/
	public void setNormalScale() {
    	CURRENT_DRIVE_SCALE = JOYSTICK_DRIVE_SCALE;
    	isLowScale = false;
    }
    public void setLowScale() {
    	CURRENT_DRIVE_SCALE = JOYSTICK_DRIVE_SCALE_LOW;
    	isLowScale = true;
    }	
	public void toggleScale(){
    	if (isLowScale) {
    		setNormalScale();
    	}
    	else {
    		setLowScale();
		}
	}	
}