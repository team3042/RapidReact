package org.usfirst.frc.team3042.robot.subsystems;

import org.usfirst.frc.team3042.lib.Log;
import org.usfirst.frc.team3042.robot.RobotMap;
import org.usfirst.frc.team3042.robot.commands.drivetrain.Drivetrain_TankDrive;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;

import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.util.sendable.SendableRegistry;

/** Drivetrain ****************************************************************
 * The drivetrain subsystem of the robot. */
public class Drivetrain extends Subsystem {
	/** Configuration Constants ***********************************************/
	private static final Log.Level LOG_LEVEL = RobotMap.LOG_DRIVETRAIN;
	private static final int LEFT_FRONT = RobotMap.CAN_LEFT_FRONT_MOTOR;
	private static final int RIGHT_FRONT = RobotMap.CAN_LEFT_BACK_MOTOR;
	private static final int LEFT_BACK = RobotMap.CAN_RIGHT_FRONT_MOTOR;
	private static final int RIGHT_BACK = RobotMap.CAN_RIGHT_BACK_MOTOR;
	private static final NeutralMode BRAKE_MODE = RobotMap.DRIVETRAIN_BRAKE_MODE;
	private static final boolean REVERSE_LEFT_MOTOR = RobotMap.REVERSE_LEFT_MOTOR;
	private static final boolean REVERSE_RIGHT_MOTOR = RobotMap.REVERSE_RIGHT_MOTOR;	
	private static final int COUNTS_PER_REVOLUTION = RobotMap.COUNTS_PER_REVOLUTION;
	private static final int FRAME_RATE = RobotMap.ENCODER_FRAME_RATE;
	private static final int TIMEOUT = RobotMap.AUTON_TIMEOUT;
	private static final int PIDIDX = RobotMap.AUTON_PIDIDX;
	private static final boolean SENSOR_PHASE_LEFT = RobotMap.SENSOR_PHASE_LEFT;
	private static final boolean SENSOR_PHASE_RIGHT = RobotMap.SENSOR_PHASE_RIGHT;

	/** Instance Variables ****************************************************/
	Log log = new Log(LOG_LEVEL, SendableRegistry.getName(this));

	WPI_TalonSRX leftFront = new WPI_TalonSRX(LEFT_FRONT);
	WPI_TalonSRX rightFront = new WPI_TalonSRX(RIGHT_FRONT);
	WPI_TalonSRX leftBack = new WPI_TalonSRX(LEFT_BACK);
	WPI_TalonSRX rightBack = new WPI_TalonSRX(RIGHT_BACK);	

	Gyro gyroscope = new ADXRS450_Gyro(); // The gyroscope sensor

	double leftPositionZero, rightPositionZero; // Zero positions of the encoders
	
	/** Drivetrain ************************************************************
	 * Setup the motor controllers for desired behavior. */
	public Drivetrain() {
		log.add("Constructor", LOG_LEVEL);
		
		initMotor(leftFront, REVERSE_LEFT_MOTOR, SENSOR_PHASE_LEFT);
		initMotor(rightFront, REVERSE_RIGHT_MOTOR, SENSOR_PHASE_RIGHT);
		initMotor(leftBack, REVERSE_LEFT_MOTOR, SENSOR_PHASE_LEFT);
		initMotor(rightBack, REVERSE_RIGHT_MOTOR, SENSOR_PHASE_RIGHT);

		leftBack.set(ControlMode.Follower, LEFT_FRONT);
		rightBack.set(ControlMode.Follower, RIGHT_FRONT);
													
		resetEncoders();
	}
	private void initMotor(WPI_TalonSRX motor, boolean reverse, boolean sensorPhase) {
		motor.setNeutralMode(BRAKE_MODE);
		motor.setInverted(reverse);
		motor.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, PIDIDX, TIMEOUT);
		motor.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, FRAME_RATE, TIMEOUT);
		motor.setSensorPhase(sensorPhase); // affects closed-loop mode
	}
	
	/** initDefaultCommand ****************************************************
	 * Set the default command for the subsystem. */
	public void initDefaultCommand() {
		setDefaultCommand(new Drivetrain_TankDrive());
	}
	
	/** Methods for setting the motors in % power mode ********************/
	public void setPower(double leftPower, double rightPower) {
		leftPower = safetyCheck(leftPower);
		rightPower = safetyCheck(rightPower);
				
		leftFront.set(ControlMode.PercentOutput, leftPower);
		rightFront.set(ControlMode.PercentOutput, rightPower);		
	}
	public void stop() {
		setPower(0.0, 0.0);
	}
	private double safetyCheck(double power) {
		power = Math.min(1.0, power);
		power = Math.max(-1.0, power);
		return power;
	}

	/** Gyroscope Methods *******************************************************/
  	public void zeroGyro() { // Zeroes the heading of the robot
    	gyroscope.reset();
	  }
	public double getAngle() { // Returns the heading of the robot
		return gyroscope.getRotation2d().getDegrees();
	}
	public double getTurnRate() { // Returns the turn rate of the robot
		return -gyroscope.getRate();
	}

	/** resetEncoders ***********************************************************/
	public void resetEncoders() {
		int leftCounts = (int)(leftFront.getSelectedSensorPosition(PIDIDX));
		leftPositionZero = countsToRev(leftCounts);
		
		int rightCounts = (int)(rightFront.getSelectedSensorPosition(PIDIDX));
		rightPositionZero = countsToRev(rightCounts);
	}
	
	/** Get the encoder position or speed *************************************
	 * Position is converted to revolutions
	 * Speed returns counts per 100ms and is converted to RPM */
	public double getLeftPosition() {
		int counts = (int)(leftFront.getSelectedSensorPosition(PIDIDX));
		return countsToRev(counts) - leftPositionZero;
	}
	public double getRightPosition() {
		int counts = (int)(rightFront.getSelectedSensorPosition(PIDIDX));
		return countsToRev(counts) - rightPositionZero;
	}
	private double countsToRev(int counts) {
		return (double)counts / COUNTS_PER_REVOLUTION;
	}
	public double getLeftSpeed() {
		int cp100ms = (int)(leftFront.getSelectedSensorVelocity(PIDIDX));
		return cp100msToRPM(cp100ms);
	}
	public double getRightSpeed() {
		int cp100ms = (int)(rightFront.getSelectedSensorVelocity(PIDIDX));
		return cp100msToRPM(cp100ms);
	}
	private double cp100msToRPM(int cp100ms) {
		return (double)cp100ms * 10.0 * 60.0 / COUNTS_PER_REVOLUTION;
	}
	
	/** rpmToF ****************************************************************
	 * Convert RPM reading into an F-Gain
	 * Note that 1023 is the native full-forward power of the talons, 
	 * equivalent to setting the power to 1.0.
	 * The speed has to be converted from rpm to encoder counts per 100ms
	 * so F = power * 1023 / speed */
	public double rpmToF(double rpm, double power) {
		//Convert to counts per 100 ms
		double speed = rpm * 4.0 * COUNTS_PER_REVOLUTION / 600.0;
		double kF = power * 1023.0 / speed;
		return kF;
	}
	public double rpmToPower(double rpm, double kF) {
		//Convert to counts per 100 ms
		double speed = rpm * 4.0 * COUNTS_PER_REVOLUTION / 600.0;
		double power = kF * speed / 1023.0;
		return power;
	}
}