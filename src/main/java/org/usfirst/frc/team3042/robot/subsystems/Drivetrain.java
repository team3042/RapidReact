package org.usfirst.frc.team3042.robot.subsystems;

import org.usfirst.frc.team3042.lib.Log;
import org.usfirst.frc.team3042.robot.RobotMap;
import org.usfirst.frc.team3042.robot.commands.drivetrain.Drivetrain_TankDrive;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.smartdashboard.SendableRegistry;

/** Drivetrain ****************************************************************
 * The drivetrain subsystem of the robot. */
public class Drivetrain extends Subsystem {
	/** Configuration Constants ***********************************************/
	private static final Log.Level LOG_LEVEL = RobotMap.LOG_DRIVETRAIN;
	private static final int CAN_LEFT_MOTOR = RobotMap.CAN_LEFT_MOTOR;
	private static final int CAN_RIGHT_MOTOR = RobotMap.CAN_RIGHT_MOTOR;
	private static final int CAN_LEFT_FOLLOWER = RobotMap.CAN_LEFT_FOLLOWER;
	private static final int CAN_RIGHT_FOLLOWER = RobotMap.CAN_RIGHT_FOLLOWER;
	private static final NeutralMode BRAKE_MODE = RobotMap.DRIVETRAIN_BRAKE_MODE;
	private static final boolean REVERSE_LEFT_MOTOR = RobotMap.REVERSE_LEFT_MOTOR;
	private static final boolean REVERSE_RIGHT_MOTOR = RobotMap.REVERSE_RIGHT_MOTOR;	
	private static final int SOLENOID_ID = RobotMap.DRIVETRAIN_SOLENOID;
	
	private static final int COUNTS_PER_REVOLUTION = RobotMap.COUNTS_PER_REVOLUTION;
	private static final int FRAME_RATE = RobotMap.ENCODER_FRAME_RATE;
	private static final int TIMEOUT = RobotMap.AUTON_TIMEOUT;
	private static final int PIDIDX = RobotMap.AUTON_PIDIDX;
	private static final boolean SENSOR_PHASE_LEFT = RobotMap.SENSOR_PHASE_LEFT;
	private static final boolean SENSOR_PHASE_RIGHT = RobotMap.SENSOR_PHASE_RIGHT;

	/** Instance Variables ****************************************************/
	Log log = new Log(LOG_LEVEL, SendableRegistry.getName(this));

	Solenoid shifter = new Solenoid(SOLENOID_ID);

	WPI_TalonSRX leftMotor = new WPI_TalonSRX(CAN_LEFT_MOTOR);
	WPI_TalonSRX rightMotor = new WPI_TalonSRX(CAN_RIGHT_MOTOR);
	WPI_TalonSRX leftFollower = new WPI_TalonSRX(CAN_LEFT_FOLLOWER);
	WPI_TalonSRX rightFollower = new WPI_TalonSRX(CAN_RIGHT_FOLLOWER);	

	Gyro gyroscope = new ADXRS450_Gyro(); // The gyroscope sensor

	DrivetrainEncoders encoders;
	DifferentialDriveOdometry odometry; // Odometry class for tracking robot posistion

	Boolean isHighGear = false;

	TalonSRX leftEncoder, rightEncoder;
	double leftPositionZero, rightPositionZero;
	
	/** Drivetrain ************************************************************
	 * Set up the talons for desired behavior. */
	public Drivetrain() {
		log.add("Constructor", LOG_LEVEL);
		
		encoders = new DrivetrainEncoders(leftMotor, rightMotor);

		initMotor(leftMotor, REVERSE_LEFT_MOTOR);
		initMotor(rightMotor, REVERSE_RIGHT_MOTOR);
		initMotor(leftFollower, REVERSE_LEFT_MOTOR);
		initMotor(rightFollower, REVERSE_RIGHT_MOTOR);

		leftFollower.set(ControlMode.Follower, CAN_LEFT_MOTOR);
		rightFollower.set(ControlMode.Follower, CAN_RIGHT_MOTOR);

		odometry = new DifferentialDriveOdometry(gyroscope.getRotation2d());

		setLowGear();
	}
	private void initMotor(WPI_TalonSRX motor, boolean reverse) {
		motor.setNeutralMode(BRAKE_MODE);
		motor.setInverted(reverse);
	}
	
	/** initDefaultCommand ****************************************************
	 * Set the default command for the subsystem. */
	public void initDefaultCommand() {
		setDefaultCommand(new Drivetrain_TankDrive());
		setDefaultCommand(null);
	}
	
	/** Methods for setting the motors in % power mode ********************/
	public void setPower(double leftPower, double rightPower) {
		leftPower = safetyCheck(leftPower);
		rightPower = safetyCheck(rightPower);
				
		leftMotor.set(ControlMode.PercentOutput, leftPower);
		rightMotor.set(ControlMode.PercentOutput, rightPower);		
	}
	public void stop() { // Stop the drivetrain
		setPower(0.0, 0.0);
	}
	private double safetyCheck(double power) {
		power = Math.min(1.0, power);
		power = Math.max(-1.0, power);
		return power;
	}

	/** Odometry Methods *******************************************************/
  	public void updateOdometry() { // Updates the field-relative position.
    	odometry.update(gyroscope.getRotation2d(), positionToMeters(encoders.getLeftPosition()), positionToMeters(encoders.getRightPosition()));
  	}
  	public void resetOdometry(Pose2d pose) { // Resets the field-relative position to a specific location.
    	odometry.resetPosition(pose, gyroscope.getRotation2d());
  	}
  	public Pose2d getPose() { // Returns the position of the robot.
    	return odometry.getPoseMeters();
	  }
	  
	// Conversion Methods: Convert to Meters
	public double positionToMeters(double position) {
		return position * Math.PI * RobotMap.WHEEL_DIAMETER / 39.3700787; // Divide by 39.3700787 to convert inches to meters
	}
	public double speedToMeters(double speed) {
		return speed / 60 * Math.PI * RobotMap.WHEEL_DIAMETER / 39.3700787; // Divide by 39.3700787 to convert inches to meters
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
	  
	/* Methods for controlling the pneumatic gearbox shifter */
	public void setHighGear(){
    	shifter.set(true);
    	isHighGear = true;
    }
    public void setLowGear(){
    	shifter.set(false);
    	isHighGear = false;
    }
    public void toggleGear(){
    	if (isHighGear){
    		setLowGear();
    	}
    	else {
    		setHighGear();
    	}
    }
	
	/** Give commands access to the drivetrain encoders ****************/
	public DrivetrainEncoders getEncoders() {
		return encoders;
	}

	/** DrivetrainEncoders ****************************************************/
	public void DrivetrainEncoders(TalonSRX leftMotor, TalonSRX rightMotor) {
		log.add("Constructor", LOG_LEVEL);
		
		leftEncoder = leftMotor;
		rightEncoder = rightMotor;
				
		initEncoder(leftEncoder, SENSOR_PHASE_LEFT);
		initEncoder(rightEncoder, SENSOR_PHASE_RIGHT);
													
		reset();
	}
	private void initEncoder(TalonSRX encoder, boolean sensorPhase) {
		encoder.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, PIDIDX, TIMEOUT);
		encoder.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, FRAME_RATE, TIMEOUT);
		encoder.setSensorPhase(sensorPhase); // affects closed-loop mode
	}

	/** reset *****************************************************************/
	public void reset() {
		int leftCounts = (int)(leftEncoder.getSelectedSensorPosition(PIDIDX));
		leftPositionZero = countsToRev(leftCounts);
		
		int rightCounts = (int)(rightEncoder.getSelectedSensorPosition(PIDIDX));
		rightPositionZero = countsToRev(rightCounts);
	}
	public void setToZero() {
		leftPositionZero = 0.0;
		rightPositionZero = 0.0;
	}
	
	/** Get the encoder position or speed *************************************
	 * Position is converted to revolutions
	 * Speed returns counts per 100ms and is converted to RPM */
	public double getLeftPosition() {
		int counts = (int)(leftEncoder.getSelectedSensorPosition(PIDIDX));
		return countsToRev(counts) - leftPositionZero;
	}
	public double getRightPosition() {
		int counts = (int)(rightEncoder.getSelectedSensorPosition(PIDIDX));
		return countsToRev(counts) - rightPositionZero;
	}
	private double countsToRev(int counts) {
		return (double)counts / COUNTS_PER_REVOLUTION;
	}
	public double getLeftSpeed() {
		int cp100ms = (int)(leftEncoder.getSelectedSensorVelocity(PIDIDX));
		return cp100msToRPM(cp100ms);
	}
	public double getRightSpeed() {
		int cp100ms = (int)(rightEncoder.getSelectedSensorVelocity(PIDIDX));
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