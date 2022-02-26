package org.usfirst.frc.team3042.robot;

import org.usfirst.frc.team3042.lib.Log;
import org.usfirst.frc.team3042.robot.commands.Climber_Ratchet;
import org.usfirst.frc.team3042.robot.commands.Climber_Run;
import org.usfirst.frc.team3042.robot.commands.Conveyor_Run;
import org.usfirst.frc.team3042.robot.commands.Intake_Intake;
import org.usfirst.frc.team3042.robot.commands.Intake_Toggle;
import org.usfirst.frc.team3042.robot.commands.autonomous.helperCommands.Drivetrain_Scale_Toggle;
import org.usfirst.frc.team3042.robot.subsystems.Drivetrain;

import edu.wpi.first.wpilibj2.command.InstantCommand;

/** OI ************************************************************************
 * This class is the glue that binds the controls on the physical operator
 * interface to the commands and command groups that allow control of the robot. */
public class OI {	
	/** Configuration Constants ***********************************************/
	private static final int USB_GAMEPAD = RobotMap.USB_GAMEPAD;
	private static final int USB_JOY_LEFT = RobotMap.USB_JOYSTICK_LEFT;
	private static final int USB_JOY_RIGHT = RobotMap.USB_JOYSTICK_RIGHT;
	private static final double JOYSTICK_DEAD_ZONE = RobotMap.JOYSTICK_DEAD_ZONE;
	private static final double TRIGGER_SPINNER_SCALE = RobotMap.TRIGGER_SPINNER_SCALE;	
	private static final int JOYSTICK_X_AXIS = Gamepad.JOY_X_AXIS;
	private static final int JOYSTICK_Y_AXIS = Gamepad.JOY_Y_AXIS;
	private static final int JOYSTICK_Z_AXIS = Gamepad.JOY_Z_AXIS;
	private static final int GAMEPAD_LEFT_TRIGGER = Gamepad.LEFT_TRIGGER;
	private static final int GAMEPAD_RIGHT_TRIGGER = Gamepad.RIGHT_TRIGGER;
	private static final double JOYSTICK_DRIVE_SCALE = RobotMap.JOYSTICK_DRIVE_SCALE;
	private static final double JOYSTICK_DRIVE_SCALE_LOW = RobotMap.JOYSTICK_DRIVE_SCALE_LOW;
	
	/** Instance Variables ****************************************************/
	Log log = new Log(RobotMap.LOG_OI, "OI");
	public Gamepad gamepad, joyLeft, joyRight;
	int driveAxisX, driveAxisY, driveAxisZ;
	Drivetrain drivetrain = Robot.drivetrain;
	public static double CURRENT_DRIVE_SCALE = JOYSTICK_DRIVE_SCALE;
	public static boolean isLowScale = false;

	/** OI ********************************************************************
	 * Assign commands to the buttons and triggers*/
	public OI() {
		log.add("OI Constructor", Log.Level.TRACE);
		
		gamepad = new Gamepad(USB_GAMEPAD);
		
		//Setup Driving Controls ///
		joyLeft = new Gamepad(USB_JOY_LEFT);
		joyRight = new Gamepad(USB_JOY_RIGHT);
		driveAxisX = JOYSTICK_X_AXIS;
		driveAxisY = JOYSTICK_Y_AXIS;
		driveAxisZ = JOYSTICK_Z_AXIS;

		joyLeft.button1.whenPressed(new InstantCommand(drivetrain::zeroGyro, drivetrain)); // Zero the gyro, this is helpful for field-oriented driving
		joyRight.button1.whenPressed(new Drivetrain_Scale_Toggle());
		joyRight.button1.whenReleased(new Drivetrain_Scale_Toggle());
		
		// Intake Controls //
		gamepad.LB.whenPressed(new Intake_Intake(1)); // run the intake
		gamepad.LB.whenReleased(new Intake_Intake(0)); // stop running the intake

		gamepad.LT.whenActive(new Intake_Intake(-1)); // reverse the intake
		gamepad.LT.whenInactive(new Intake_Intake(0)); // stop reversing the intake

		gamepad.A.whenPressed(new Intake_Toggle()); // extend or retract the intake

		// Climber Controls //
		gamepad.POVUp.whenActive(new Climber_Run(1)); // raise the climber
		gamepad.POVUp.whenInactive(new Climber_Run(0)); // stop the climber
		gamepad.POVDown.whenActive(new Climber_Run(-1)); // lower the climber
		gamepad.POVDown.whenInactive(new Climber_Run(0)); // stop the climber

		gamepad.X.whenPressed(new Climber_Ratchet()); // extend or retract the climber ratchet

		// Conveyor Controls //
		gamepad.RB.whenPressed(new Conveyor_Run(1)); // run the converyor
		gamepad.RB.whenReleased(new Conveyor_Run(0)); // stop running the converyor

		gamepad.RT.whenActive(new Conveyor_Run(0.5)); // run the converyor slowly
		gamepad.RT.whenInactive(new Conveyor_Run(0)); // stop running the conveyor slowly

		gamepad.B.whenPressed(new Conveyor_Run(-0.5)); // reverse the converyor
		gamepad.B.whenReleased(new Conveyor_Run(0)); // stop reversing the converyor
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
		joystickValue = checkDeadZone(joystickValue);
		joystickValue *= CURRENT_DRIVE_SCALE;
		return joystickValue;
	}
	private double checkDeadZone(double joystickValue) {
		if (Math.abs(joystickValue) < JOYSTICK_DEAD_ZONE) {
			joystickValue = 0.0;
		}
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
	
	/** Access the POV value *******************************************/
	public int getPOV() {
		return gamepad.getPOV();
	}
	
	/** Access the Trigger Values **************************************/
	public double getTriggerDifference() {
		double leftTrigger = gamepad.getRawAxis(GAMEPAD_LEFT_TRIGGER);
		double rightTrigger = gamepad.getRawAxis(GAMEPAD_RIGHT_TRIGGER);
		return (rightTrigger - leftTrigger) * TRIGGER_SPINNER_SCALE;
	}
}