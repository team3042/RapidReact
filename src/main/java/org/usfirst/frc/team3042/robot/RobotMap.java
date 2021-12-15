package org.usfirst.frc.team3042.robot;

import org.usfirst.frc.team3042.lib.Log;

import com.ctre.phoenix.motorcontrol.NeutralMode;

/** RobotMap ******************************************************************
 * The robot configuration file. */
public class RobotMap {	
	/** Robot Size Parameters *************************************************
	 * The units of the wheel diameter determine the units of the position 
	 * and speed closed-loop commands. For example, if the diameter is given 
	 * in inches, position will be in inches and speed in inches per second. */
	public static final double WHEEL_DIAMETER = 5.5; //TODO: Measure this on the actual 2022 robot!
	public static final double ROBOT_WIDTH = 21.5; //TODO: Measure this on the actual 2022 robot!
	
	/** CAN ID numbers ********************************************************/
	public static final int CAN_LEFT_MOTOR 	= 33; //TODO: Determine this CAN ID on the actual 2022 robot!
	public static final int CAN_RIGHT_MOTOR = 29; //TODO: Determine this CAN ID on the actual 2022 robot!
	public static final int CAN_LEFT_FOLLOWER = 30; //TODO: Determine this CAN ID on the actual 2022 robot!
	public static final int CAN_RIGHT_FOLLOWER = 28; //TODO: Determine this CAN ID on the actual 2022 robot!
	public static final int CAN_INTAKE = 13; //TODO: Determine this CAN ID on the actual 2022 robot!

	/** PCM channels **********************************************************/
	
	/** DIO channels **********************************************************/
	
	/** Drivetrain Settings ***************************************************/
	public static final NeutralMode DRIVETRAIN_BRAKE_MODE = NeutralMode.Brake;
	public static final boolean REVERSE_LEFT_MOTOR = false; //TODO: Determine this setting for the actual 2022 robot!
	public static final boolean REVERSE_RIGHT_MOTOR = true; //TODO: Determine this setting for the actual 2022 robot!
	public static final double ACCELERATION_MAX = 1.5; // Maximum Acceleration given in power(volts) per second
	public static final double kF_DRIVE_LEFT = 0.1817180616740088;
	public static final double kF_DRIVE_RIGHT = 0.16686239968682717;
	public static final int COUNTS_PER_REVOLUTION = 1440; // In quadrature mode, actual counts will be 4x the cycles; e.g., 360 -> 1440
	public static final int ENCODER_FRAME_RATE = 10; // How often the encoders update on the CAN, in milliseconds
	public static final boolean SENSOR_PHASE_LEFT = false;
	public static final boolean SENSOR_PHASE_RIGHT = false;
	public static final int AUTON_TIMEOUT = 0; // timeout in ms; set to zero
	public static final int AUTON_PIDIDX = 0; // used for cascading PID; set to zero

	/** Drivetrain Gyro Drive Settings ****************************************/
	public static final double kP_GYRO = 0.026;
	public static final double kI_GYRO = 0.0;
	public static final double kD_GYRO = 0.017;
	public static final double ANGLE_TOLERANCE = 2.0;
	public static final double MAX_POWER_GYRO = 0.5;

	/** Intake Settings *******************************************************/
	public static final boolean REVERSE_INTAKE = true; //TODO: Determine this setting for the actual 2022 robot!
	public static final NeutralMode INTAKE_BRAKE_MODE = NeutralMode.Brake;
	public static final double INTAKE_POWER = 0.75; // How much power (as a %) to give the intake

	/** OI Settings ***********************************************************/
	public static final boolean USE_JOYSTICKS = true;
	public static final double JOYSTICK_DRIVE_SCALE = 1.1; // Determines driving speed of the robot
	public static final double JOYSTICK_DRIVE_SCALE_LOW = 0.25; // Determines driving speed of the robot when in slow mode
	public static final double TRIGGER_SPINNER_SCALE = 0.1;
	public static final double JOYSTICK_DEAD_ZONE = 0.0;
	
	/** USB ports *************************************************************/					
	public static final int USB_JOYSTICK_LEFT 	= 0;
	public static final int USB_JOYSTICK_RIGHT 	= 1;
	public static final int USB_GAMEPAD 		= 2;

	/** Logger Settings *******************************************************/
	public static final String 		LOG_FILE_FORMAT 					= "yyyy-MM-dd-hhmmss";
	public static final String 		LOG_TIME_FORMAT 					= "hh:mm:ss:SSS";
	public static final String 		LOG_DIRECTORY_PATH 					= "/home/lvuser/logs/";
	public static final String 		LOG_TIME_ZONE 						= "America/Chicago";
	public static final boolean 	LOG_TO_CONSOLE 						= true;
	public static final boolean 	LOG_TO_FILE 						= false;
	public static final Log.Level 	LOG_GLOBAL 							= Log.Level.DEBUG;
	public static final Log.Level 	LOG_ROBOT 							= Log.Level.TRACE;
	public static final Log.Level	LOG_OI 								= Log.Level.TRACE;
	public static final Log.Level	LOG_AXIS_TRIGGER 					= Log.Level.ERROR;
	public static final Log.Level	LOG_POV_BUTTON						= Log.Level.ERROR;
	/** Subsystems ************************************************************/
	public static final Log.Level	LOG_DRIVETRAIN						= Log.Level.TRACE;
	public static final Log.Level	LOG_INTAKE							= Log.Level.DEBUG;
}