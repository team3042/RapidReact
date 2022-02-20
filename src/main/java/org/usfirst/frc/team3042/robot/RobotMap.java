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
	public static final double TRACK_WIDTH = 0.57785; // Distance between centers of right and left wheels on robot (in meters) //TODO: We might need to replace this with the calculated track width from SysID
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

	/** PCM channels **********************************************************/
	public static final int INTAKE_SOLENOID = 1;
	
	/** DIO channels **********************************************************/
	public static final int DIO_LIMITSWITCH = 0; //TODO Find this value

	/** Climber Settings ******************************************************/
	public static final boolean REVERSE_RIGHT_CLIMBER = false;
	public static final boolean REVERSE_LEFT_CLIMBER = false;
	public static final NeutralMode CLIMBER_BRAKE_MODE = NeutralMode.Brake;
	public static final double CLIMBER_POWER = 0.85; // How much power (as a %) to give the climber
	
	/** Conveyor Settings *****************************************************/
	public static final boolean REVERSE_TOP_CONVEYOR = false;
	public static final boolean REVERSE_RIGHT_CONVEYOR = true;
	public static final boolean REVERSE_LEFT_CONVEYOR = true;
	public static final NeutralMode CONVEYOR_BRAKE_MODE = NeutralMode.Brake;
	public static final double CONVEYOR_POWER = 0.65; // How much power (as a %) to give the conveyor
	public static final double CONVEYOR_ADVANCE_DURATION = 1.0; // How much time in seconds that the conveyor runs //TODO: tune this value through testing!

	/** Drivetrain Settings ***************************************************/
	public static final IdleMode DRIVETRAIN_BRAKE_MODE = IdleMode.kBrake;
	public static final boolean REVERSE_LEFT_FRONT_MOTOR = false;
	public static final boolean REVERSE_RIGHT_FRONT_MOTOR = false;
	public static final boolean REVERSE_LEFT_BACK_MOTOR = true;
	public static final boolean REVERSE_RIGHT_BACK_MOTOR = true;
	public static final double VELOCITY_MAX_MPS = 4; // Maximum velocity in meters/second TODO: We'll probably want to tune this value
	public static final double ACCELERATION_MAX_MPS = 2; // Maximum acceleration in meters/second squared //TODO: We'll probably want to tune this value
	public static final double kP_FRONT_RIGHT_VELOCITY = 0.0010269;
	public static final double kP_FRONT_LEFT_VELOCITY = 0.0010269;
	public static final double kP_BACK_RIGHT_VELOCITY = 0.0010269;
	public static final double kP_BACK_LEFT_VELOCITY = 0.0010269;
	public static final double kP_X_CONTROLLER = 9.6421; //TODO: Tune this using guess-and-check after our drivetrain has been characterized!
    public static final double kP_Y_CONTROLLER = 9.6421; //TODO: Tune this using guess-and-check after our drivetrain has been characterized!
    public static final double kP_THETA_CONTROLLER = 9.6421; //TODO: Tune this using guess-and-check after our drivetrain has been characterized!
	public static final double kMAX_ANGULAR_SPEED_RADIANS_PER_SECOND = Math.PI; //TODO: Tune this??? (not sure yet if we'll need to change this value)
	public static final double kMAX_ANGULAR_ACCELERATION_RADIANS_PER_SECOND_SQUARED = Math.PI; //TODO: Tune this??? (not sure yet if we'll need to change this value)

	/** Drivetrain Gyro Drive Settings ****************************************/
	public static final double kP_GYRO = 0.01;
	public static final double kI_GYRO = 0.0;
	public static final double kD_GYRO = 0.015;
	public static final double ANGLE_TOLERANCE = 2.0;
	public static final double MAX_POWER_GYRO = 0.4;

	/** Intake Settings *******************************************************/
	public static final boolean REVERSE_INTAKE = true;
	public static final NeutralMode INTAKE_BRAKE_MODE = NeutralMode.Brake;
	public static final double INTAKE_POWER = 0.9; // How much power (as a %) to give the intake

	/** OI Settings ***********************************************************/
	public static final boolean USE_JOYSTICKS = true;
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
	public static final Log.Level	LOG_CLIMBER							= Log.Level.DEBUG;
	public static final Log.Level	LOG_CONVEYOR						= Log.Level.DEBUG;
	public static final Log.Level	LOG_DRIVETRAIN						= Log.Level.TRACE;
	public static final Log.Level	LOG_INTAKE							= Log.Level.DEBUG;
}