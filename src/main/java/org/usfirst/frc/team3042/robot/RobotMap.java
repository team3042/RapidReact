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
	public static final double WHEEL_DIAMETER = 5.5;
	public static final double ROBOT_WIDTH = 21.5;
	
	/** CAN ID numbers ********************************************************/
	public static final int CAN_LEFT_MOTOR 	= 19;
	public static final int CAN_RIGHT_MOTOR = 17;
	public static final int CAN_LEFT_FOLLOWER = 2;
	public static final int CAN_RIGHT_FOLLOWER = 15;
	public static final int CAN_TURRET = 26;
	public static final int CAN_INTAKE = 13;
	public static final int CAN_SHOOTER = 32;
	public static final int CAN_UPPER_CONVEYOR = 11;
	public static final int CAN_LOWER_CONVEYOR = 0;

	/** PCM channels **********************************************************/
	public static final int DRIVETRAIN_SOLENOID = 0;
	
	/** Conveyor Settings (Upper) *********************************************/
	public static final boolean REVERSE_UPPER_CONVEYOR = true;
	public static final NeutralMode UPPER_CONVEYOR_BRAKE_MODE = NeutralMode.Brake;
	public static final double UPPER_CONVEYOR_POWER = 0.9; // How much power (as a %) to give the upper conveyor
	/** Conveyor Settings (Lower) *********************************************/
	public static final boolean REVERSE_LOWER_CONVEYOR = true;
	public static final NeutralMode LOWER_CONVEYOR_BRAKE_MODE = NeutralMode.Brake;
	public static final double LOWER_CONVEYOR_POWER = 0.9; // How much power (as a %) to give the lower conveyor
	public static final double CONVEYOR_ADVANCE_DURATION = 0.25; // How long (in seconds) to run the lower conveyor when a power cell is intaked
	public static final double POWER_CELL_DISTANCE = 5; // If the ultrasonic sensor returns a distance smaller than this (units is inches) then there is a power cell in front of it
	
	/** DIO channels **********************************************************/
	public static final int DIO_ULTRASONIC_PING = 8;
	public static final int DIO_ULTRASONIC_ECHO = 9;
	
	/** Drivetrain Settings ***************************************************/
	public static final NeutralMode DRIVETRAIN_BRAKE_MODE = NeutralMode.Brake;
	public static final boolean REVERSE_LEFT_MOTOR = false;
	public static final boolean REVERSE_RIGHT_MOTOR = true;
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
	public static final boolean REVERSE_INTAKE = true;
	public static final NeutralMode INTAKE_BRAKE_MODE = NeutralMode.Brake;
	public static final double INTAKE_POWER = 0.9; // How much power (as a %) to give the intake

	/** OI Settings ***********************************************************/
	public static final boolean USE_JOYSTICKS = true;
	public static final double JOYSTICK_DRIVE_SCALE = 1.1; // Determines driving speed of the robot
	public static final double JOYSTICK_DRIVE_SCALE_LOW = 0.25; // Determines driving speed of the robot when in slow mode
	public static final double TRIGGER_SPINNER_SCALE = 0.1;
	public static final double JOYSTICK_DEAD_ZONE = 0.0;
	
	/** Shooter Settings ******************************************************/
	public static final boolean REVERSE_SHOOTER = true; // Used to reverse the direction of the shooter motor
	public static final double SHOOTER_VELOCITY = 3000; // Shooter velocity in RPM
	public static final int SHOOTER_TIMEOUT = 0; // timeout in ms; set to zero
	public static final int SHOOTER_PIDIDX = 0; // used for cascading PID; set to zero
	public static final double kP_SHOOTER_SPEED = 8; // Proportional term
	public static final int SHOOTER_ENCODER_COUNTS_PER_REV = 4096; // The number of encoder counts equal to one full revolution of the encoder
	
	/** Turret Settings *******************************************************/
	public static final boolean REVERSE_TURRET = false;
	public static final NeutralMode TURRET_BRAKE_MODE = NeutralMode.Brake;
	public static final int TURRET_TIMEOUT = 0; // timeout in ms; set to zero
	public static final int TURRET_PIDIDX = 0; // used for cascading PID; set to zero
	public static final double kP_TURRET = 0.016; // // P constant for the target-tracking PID loop
	public static final double kI_TURRET = 0.004; // I constant for the target-tracking PID loop
	public static final double kD_TURRET = 0.0; // D constant for the target-tracking PID loop
	public static final double TURRET_MAX_POWER = 0.4; // The maximum power (as a %) the turret will be given when running the target-tracking PID loop
	public static final double TURRET_MANUAL_POWER = 0.2; // How much power (as a %) to give the turret when using manual control
	public static final int TURRET_MAX_ANGLE = 80; // The maximum angle the turret can turn to in either direction (to prevent tangling of wires)
	public static final double TURRET_SEARCH_POWER = 0.6; // The speed at which the turret zips around to the other side when the max angle is reached, and also the speed at which it searches for the target if the Limelight loses it
	public static final double TURRET_ANGLE_TOLERANCE = 0.25; // If the angle of error to the target is less than this value the PID Loop will not make any corrections
	public static final int TURRET_ENCODER_FRAME_RATE = 10;
	public static final int TURRET_ENCODER_COUNTS_PER_REV = 1440; // The number of encoder counts equal to one full revolution of the encoder 
	public static final boolean TURRET_SENSOR_PHASE = false;
	
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
	public static final Log.Level	LOG_LIMELIGHT						= Log.Level.DEBUG;
	public static final Log.Level	LOG_TURRET							= Log.Level.DEBUG;
	public static final Log.Level	LOG_INTAKE							= Log.Level.DEBUG;
	public static final Log.Level	LOG_SHOOTER							= Log.Level.DEBUG;
	public static final Log.Level	LOG_LOWER_CONVEYOR					= Log.Level.DEBUG;
	public static final Log.Level	LOG_UPPER_CONVEYOR					= Log.Level.DEBUG;
}