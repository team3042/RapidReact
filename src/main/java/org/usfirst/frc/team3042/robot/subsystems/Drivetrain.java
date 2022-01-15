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
import edu.wpi.first.wpilibj.drive.MecanumDrive;
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
	private static final boolean REVERSE_LEFT_FRONT = RobotMap.REVERSE_LEFT_FRONT_MOTOR;
	private static final boolean REVERSE_RIGHT_FRONT = RobotMap.REVERSE_RIGHT_FRONT_MOTOR;
	private static final boolean REVERSE_LEFT_BACK = RobotMap.REVERSE_LEFT_BACK_MOTOR;
	private static final boolean REVERSE_RIGHT_BACK = RobotMap.REVERSE_RIGHT_BACK_MOTOR;
	private static final int COUNTS_PER_REVOLUTION = RobotMap.COUNTS_PER_REVOLUTION;
	private static final int FRAME_RATE = RobotMap.ENCODER_FRAME_RATE;
	private static final int TIMEOUT = RobotMap.AUTON_TIMEOUT;
	private static final int PIDIDX = RobotMap.AUTON_PIDIDX;
	private static final boolean SENSOR_PHASE_LEFT_FRONT = RobotMap.SENSOR_PHASE_LEFT_FRONT;
	private static final boolean SENSOR_PHASE_RIGHT_FRONT = RobotMap.SENSOR_PHASE_RIGHT_FRONT;
	private static final boolean SENSOR_PHASE_LEFT_BACK = RobotMap.SENSOR_PHASE_LEFT_BACK;
	private static final boolean SENSOR_PHASE_RIGHT_BACK = RobotMap.SENSOR_PHASE_RIGHT_BACK;

	/** Instance Variables ****************************************************/
	Log log = new Log(LOG_LEVEL, SendableRegistry.getName(this));

	WPI_TalonSRX leftFront = new WPI_TalonSRX(LEFT_FRONT);
	WPI_TalonSRX rightFront = new WPI_TalonSRX(RIGHT_FRONT);
	WPI_TalonSRX leftBack = new WPI_TalonSRX(LEFT_BACK);
	WPI_TalonSRX rightBack = new WPI_TalonSRX(RIGHT_BACK);	

	Gyro gyroscope = new ADXRS450_Gyro(); // The gyroscope sensor
	MecanumDrive robotDrive = new MecanumDrive(leftFront, rightFront, leftBack, rightBack);;

	double leftFrontPositionZero, rightFrontPositionZero, leftBackPositionZero, rightBackPositionZero; // Zero positions of the encoders
	
	/** Drivetrain ************************************************************
	 * Setup the motor controllers for desired behavior. */
	public Drivetrain() {
		log.add("Constructor", LOG_LEVEL);
		
		initMotor(leftFront, REVERSE_LEFT_FRONT, SENSOR_PHASE_LEFT_FRONT);
		initMotor(rightFront, REVERSE_RIGHT_FRONT, SENSOR_PHASE_RIGHT_FRONT);
		initMotor(leftBack, REVERSE_LEFT_BACK, SENSOR_PHASE_LEFT_BACK);
		initMotor(rightBack, REVERSE_RIGHT_BACK, SENSOR_PHASE_RIGHT_BACK);
		
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
	public void setPower(double leftFrontPower, double rightFrontPower, double leftBackPower, double rightBackPower) {
		leftFrontPower = safetyCheck(leftFrontPower);
		rightFrontPower = safetyCheck(rightFrontPower);
		leftBackPower = safetyCheck(leftBackPower);
		rightBackPower = safetyCheck(rightBackPower);
				
		leftFront.set(ControlMode.PercentOutput, leftFrontPower);
		rightFront.set(ControlMode.PercentOutput, rightFrontPower);
		leftBack.set(ControlMode.PercentOutput, leftBackPower);
		rightBack.set(ControlMode.PercentOutput, rightBackPower);		
	}
	public void stop() {
		setPower(0.0, 0.0, 0.0, 0.0);
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
		int leftFrontCounts = (int)(leftFront.getSelectedSensorPosition(PIDIDX));
		leftFrontPositionZero = countsToRev(leftFrontCounts);
		
		int rightFrontCounts = (int)(rightFront.getSelectedSensorPosition(PIDIDX));
		rightFrontPositionZero = countsToRev(rightFrontCounts);

		int leftBackCounts = (int)(leftBack.getSelectedSensorPosition(PIDIDX));
		leftBackPositionZero = countsToRev(leftBackCounts);
		
		int rightBackCounts = (int)(rightBack.getSelectedSensorPosition(PIDIDX));
		rightBackPositionZero = countsToRev(rightBackCounts);
	}
	
	/** Get the encoder position or speed *************************************
	 * Position is converted to revolutions
	 * Speed returns counts per 100ms and is converted to RPM */
	public double getLeftFrontPosition() {
		int counts = (int)(leftFront.getSelectedSensorPosition(PIDIDX));
		return countsToRev(counts) - leftFrontPositionZero;
	}
	public double getRightFrontPosition() {
		int counts = (int)(rightFront.getSelectedSensorPosition(PIDIDX));
		return countsToRev(counts) - rightFrontPositionZero;
	}
	public double getLeftBackPosition() {
		int counts = (int)(leftBack.getSelectedSensorPosition(PIDIDX));
		return countsToRev(counts) - leftBackPositionZero;
	}
	public double getRightBackPosition() {
		int counts = (int)(rightBack.getSelectedSensorPosition(PIDIDX));
		return countsToRev(counts) - rightBackPositionZero;
	}

	public double getLeftFrontSpeed() {
		int cp100ms = (int)(leftFront.getSelectedSensorVelocity(PIDIDX));
		return cp100msToRPM(cp100ms);
	}
	public double getRightFrontSpeed() {
		int cp100ms = (int)(rightFront.getSelectedSensorVelocity(PIDIDX));
		return cp100msToRPM(cp100ms);
	}
	public double getLeftBackSpeed() {
		int cp100ms = (int)(leftBack.getSelectedSensorVelocity(PIDIDX));
		return cp100msToRPM(cp100ms);
	}
	public double getRightBackSpeed() {
		int cp100ms = (int)(rightBack.getSelectedSensorVelocity(PIDIDX));
		return cp100msToRPM(cp100ms);
	}

	private double countsToRev(int counts) {
		return (double)counts / COUNTS_PER_REVOLUTION;
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