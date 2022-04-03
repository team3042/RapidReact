package org.usfirst.frc.team3042.robot;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.revrobotics.CANSparkMax.IdleMode;

import org.usfirst.frc.team3042.lib.Log;

/** RobotMap ******************************************************************
 * The robot configuration file. */
public class RobotMap {	
	/** Robot Size Parameters *************************************************
	 * The units of the wheel diameter determine the units of the position 
	 * and speed closed-loop commands. For example, if the diameter is given 
	 * in inches, position will be in inches and speed in inches per second. */
	public static final double WHEEL_DIAMETER = 6.0; // Measured in inches (0.1524 m)
	public static final double TRACK_WIDTH = 0.57785; // Distance between centers of right and left wheels on robot (in meters)
    public static final double WHEEL_BASE = 0.517525; // Distance between centers of front and back wheels on robot (in meters) 

	/** CAN ID numbers ********************************************************/
	public static final int CAN_TOP_CONVEYOR = 15;
	public static final int CAN_RIGHT_CONVEYOR = 32;
	public static final int CAN_LEFT_CONVEYOR = 25;
	public static final int CAN_RIGHT_CLIMBER = 13;
	public static final int CAN_LEFT_CLIMBER = 27;
	public static final int CAN_LEFT_FRONT_MOTOR = 3;
	public static final int CAN_RIGHT_FRONT_MOTOR = 4;
	public static final int CAN_LEFT_BACK_MOTOR = 5;
	public static final int CAN_RIGHT_BACK_MOTOR = 2;
	public static final int CAN_INTAKE = 8;
	public static final int CAN_TRAVERSAL_MOTOR = 10;

	/** PCM channels **********************************************************/
	public static final int INTAKE_SOLENOID = 1;
	public static final int CLIMBER_SOLENOID = 2; 
	
	/** DIO channels **********************************************************/
	public static final int DIO_LIMITSWITCH_CONVEYOR = 1;
	public static final int DIO_LIMITSWITCH_CLIMBER_RIGHT = 3;
	public static final int DIO_LIMITSWITCH_CLIMBER_LEFT = 5;

	/** Climber Settings ******************************************************/
	public static final boolean REVERSE_RIGHT_CLIMBER = true;
	public static final boolean REVERSE_LEFT_CLIMBER = false;
	public static final boolean REVERSE_TRAVERSAL_CLIMBER = false;
	public static final NeutralMode CLIMBER_BRAKE_MODE = NeutralMode.Brake;
	public static final IdleMode TRAVERSAL_CLIMBER_BRAKE_MODE = IdleMode.kBrake;
	public static final double CLIMBER_POWER = 0.7; // How much power (as a %) to give the climbing hooks
	public static final double TRAVERSAL_CLIMBER_POWER = 0.7; // How much power (as a %) to give the traversal climber
	public static final double TRAVERSAL_GOAL_POSITION = 120;
	
	/** Conveyor Settings *****************************************************/
	public static final boolean REVERSE_TOP_CONVEYOR = false;
	public static final boolean REVERSE_RIGHT_CONVEYOR = true;
	public static final boolean REVERSE_LEFT_CONVEYOR = false;
	public static final NeutralMode CONVEYOR_BRAKE_MODE = NeutralMode.Brake;
	public static final double CONVEYOR_POWER = 0.8; // How much power (as a %) to give the conveyor
	public static final double CONVEYOR_ADVANCE_POWER = 0.4; // How much power (as a %) to give the conveyor

	/** Drivetrain Settings ***************************************************/
	public static final IdleMode DRIVETRAIN_BRAKE_MODE = IdleMode.kBrake;
	public static final double JOYSTICK_DRIVE_SCALE = 1; // Determines the max driving speed of the robot
	public static final double JOYSTICK_DRIVE_SCALE_LOW = 0.2; // Determines driving speed of the robot when in slow mode
	public static final boolean REVERSE_LEFT_FRONT_MOTOR = false;
	public static final boolean REVERSE_RIGHT_FRONT_MOTOR = false;
	public static final boolean REVERSE_LEFT_BACK_MOTOR = true;
	public static final boolean REVERSE_RIGHT_BACK_MOTOR = true;
	public static final double VELOCITY_MAX_MPS = 4;
	public static final double ACCELERATION_MAX_MPS = 2;
	public static final double kP_FRONT_RIGHT_VELOCITY = 0.0010269;
	public static final double kP_FRONT_LEFT_VELOCITY = 0.0010269;
	public static final double kP_BACK_RIGHT_VELOCITY = 0.0010269;
	public static final double kP_BACK_LEFT_VELOCITY = 0.0010269;
	public static final double kP_X_CONTROLLER = 9.6421;
    public static final double kP_Y_CONTROLLER = 9.6421;
    public static final double kP_THETA_CONTROLLER = 9.6421;
	public static final double kMAX_ANGULAR_SPEED_RADIANS_PER_SECOND = 2*Math.PI;
	public static final double kMAX_ANGULAR_ACCELERATION_RADIANS_PER_SECOND_SQUARED = 2*Math.PI;

	/** Drivetrain Gyro Drive Settings ****************************************/
	public static final double kP_GYRO = 0.01;
	public static final double kI_GYRO = 0.0;
	public static final double kD_GYRO = 0.015;
	public static final double ANGLE_TOLERANCE = 2.0;
	public static final double MAX_POWER_GYRO = 0.4;

	/** Intake Settings *******************************************************/
	public static final boolean REVERSE_INTAKE = true;
	public static final NeutralMode INTAKE_BRAKE_MODE = NeutralMode.Brake;
	public static final double INTAKE_POWER = 0.95; // How much power (as a %) to give the intake	
	
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
	public static final Log.Level	LOG_CLIMBER							= Log.Level.DEBUG;
	public static final Log.Level	LOG_CLIMBER_TRAVERSAL				= Log.Level.DEBUG;
	public static final Log.Level	LOG_CONVEYOR						= Log.Level.DEBUG;
	public static final Log.Level	LOG_DRIVETRAIN						= Log.Level.TRACE;
	public static final Log.Level	LOG_INTAKE							= Log.Level.DEBUG;
}