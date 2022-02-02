package org.usfirst.frc.team3042.robot.subsystems;

import org.usfirst.frc.team3042.lib.Log;
import org.usfirst.frc.team3042.robot.RobotMap;
import org.usfirst.frc.team3042.robot.commands.Drivetrain_MecanumDrive;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.ADIS16470_IMU;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.MecanumDriveKinematics;
import edu.wpi.first.math.kinematics.MecanumDriveOdometry;
import edu.wpi.first.math.kinematics.MecanumDriveWheelSpeeds;
import edu.wpi.first.util.sendable.SendableRegistry;

/** Drivetrain ****************************************************************
 * The drivetrain subsystem of the robot. */
public class Drivetrain extends SubsystemBase {
	/** Configuration Constants ***********************************************/
	private static final Log.Level LOG_LEVEL = RobotMap.LOG_DRIVETRAIN;
	private static final IdleMode BRAKE_MODE = RobotMap.DRIVETRAIN_BRAKE_MODE;
	private static final int COUNTS_PER_REVOLUTION = RobotMap.COUNTS_PER_REVOLUTION;
	private static final boolean REVERSE_LEFT_FRONT = RobotMap.REVERSE_LEFT_FRONT_MOTOR;
	private static final boolean REVERSE_RIGHT_FRONT = RobotMap.REVERSE_RIGHT_FRONT_MOTOR;
	private static final boolean REVERSE_LEFT_BACK = RobotMap.REVERSE_LEFT_BACK_MOTOR;
	private static final boolean REVERSE_RIGHT_BACK = RobotMap.REVERSE_RIGHT_BACK_MOTOR;
	private static final int CAN_LEFT_FRONT_MOTOR = RobotMap.CAN_LEFT_FRONT_MOTOR;
	private static final int CAN_RIGHT_FRONT_MOTOR = RobotMap.CAN_RIGHT_FRONT_MOTOR;
	private static final int CAN_LEFT_BACK_MOTOR = RobotMap.CAN_LEFT_BACK_MOTOR;
	private static final int CAN_RIGHT_BACK_MOTOR = RobotMap.CAN_RIGHT_BACK_MOTOR;
	private static final double TRACK_WIDTH = RobotMap.TRACK_WIDTH;
	private static final double WHEEL_BASE = RobotMap.WHEEL_BASE;

	private static final SimpleMotorFeedforward kFeedforward = new SimpleMotorFeedforward(0, 0, 0); //TODO Find these characterization values!
	private final PIDController frontLeftPIDController = new PIDController(RobotMap.kP_FRONT_LEFT_VELOCITY, 0, 0);
  	private final PIDController frontRightPIDController = new PIDController(RobotMap.kP_FRONT_RIGHT_VELOCITY, 0, 0);
  	private final PIDController backLeftPIDController = new PIDController(RobotMap.kP_BACK_LEFT_VELOCITY, 0, 0);
  	private final PIDController backRightPIDController = new PIDController(RobotMap.kP_BACK_RIGHT_VELOCITY, 0, 0);

	private static final MecanumDriveKinematics kDriveKinematics =
		new MecanumDriveKinematics(new Translation2d(WHEEL_BASE / 2, TRACK_WIDTH / 2), 
								   new Translation2d(WHEEL_BASE / 2, -TRACK_WIDTH / 2), 
								   new Translation2d(-WHEEL_BASE / 2, TRACK_WIDTH / 2), 
								   new Translation2d(-WHEEL_BASE / 2, -TRACK_WIDTH / 2));

	/** Instance Variables ****************************************************/
	Log log = new Log(LOG_LEVEL, SendableRegistry.getName(this));

	CANSparkMax leftFront = new CANSparkMax(CAN_LEFT_FRONT_MOTOR, MotorType.kBrushless);
	CANSparkMax rightFront = new CANSparkMax(CAN_RIGHT_FRONT_MOTOR, MotorType.kBrushless);
	CANSparkMax leftBack = new CANSparkMax(CAN_LEFT_BACK_MOTOR, MotorType.kBrushless);
	CANSparkMax rightBack = new CANSparkMax(CAN_RIGHT_BACK_MOTOR, MotorType.kBrushless);	

	ADIS16470_IMU gyroscope = new ADIS16470_IMU(); // The gyroscope sensor
	MecanumDrive robotDrive = new MecanumDrive(leftFront, rightFront, leftBack, rightBack);

	MecanumDriveOdometry odometry = new MecanumDriveOdometry(kDriveKinematics, Rotation2d.fromDegrees(-gyroscope.getAngle()));

	double leftFrontPositionZero, rightFrontPositionZero, leftBackPositionZero, rightBackPositionZero; // Zero positions of the encoders
	
	/** Drivetrain ************************************************************
	 * Setup the motor controllers for desired behavior. */
	public Drivetrain() {
		log.add("Constructor", LOG_LEVEL);

		/** The RestoreFactoryDefaults method is used to reset the configuration parameters
        * of the SPARK MAX to their factory default state to ensure consistent operation */
		leftFront.restoreFactoryDefaults();
		rightFront.restoreFactoryDefaults();
		leftBack.restoreFactoryDefaults();
		rightBack.restoreFactoryDefaults();
		
		initMotor(leftFront, REVERSE_LEFT_FRONT);
		initMotor(rightFront, REVERSE_RIGHT_FRONT);
		initMotor(leftBack, REVERSE_LEFT_BACK);
		initMotor(rightBack, REVERSE_RIGHT_BACK);
		
		resetEncoders();
	}
	private void initMotor(CANSparkMax motor, boolean reverse) {
		motor.setIdleMode(BRAKE_MODE);
		motor.setInverted(reverse);
	}

	/** initDefaultCommand ****************************************************
	 * Set the default command for the subsystem. */
	public void initDefaultCommand() {
		setDefaultCommand(new Drivetrain_MecanumDrive()); //TODO: Change this to Drivetrain_FieldOriented when we are ready to test it :)
	}
	
	/** Methods for setting the motors in % power mode ********************/
	public void setPower(double leftFrontPower, double rightFrontPower, double leftBackPower, double rightBackPower) {
		leftFrontPower = safetyCheck(leftFrontPower);
		rightFrontPower = safetyCheck(rightFrontPower);
		leftBackPower = safetyCheck(leftBackPower);
		rightBackPower = safetyCheck(rightBackPower);
				
		leftFront.set(leftFrontPower);
		rightFront.set(rightFrontPower);
		leftBack.set(leftBackPower);
		rightBack.set(rightBackPower);		
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
	public double getGyroAngle() { // Returns the heading of the robot
		return gyroscope.getAngle();
	}
	public double getTurnRate() { // Returns the turn rate of the robot
		return -gyroscope.getRate();
	}

	/** Odometry Methods *******************************************************/
	public Rotation2d getRotation2d() {
		return Rotation2d.fromDegrees(-gyroscope.getAngle());
	}
	public void resetOdometry(Pose2d pose) {
		odometry.resetPosition(pose, this.getRotation2d());
	}
	public Pose2d getPose() {
		return odometry.getPoseMeters();
	}
	public MecanumDriveKinematics getkDriveKinematics() {
		return kDriveKinematics;	
	}

	@Override
	public void periodic() {
	  // Update the odometry in the periodic block
	  odometry.update( this.getRotation2d(), new MecanumDriveWheelSpeeds(
		leftFront.getEncoder().getVelocity() / 10.71,
		leftBack.getEncoder().getVelocity() / 10.71,
		rightFront.getEncoder().getVelocity() / 10.71,
		rightBack.getEncoder().getVelocity() / 10.71));
	}

	/** resetEncoders ***********************************************************/
	public void resetEncoders() {
		leftFrontPositionZero = (int)(leftFront.getEncoder().getPosition() / 10.71);		
		rightFrontPositionZero = (int)(rightFront.getEncoder().getPosition() / 10.71);
		leftBackPositionZero = (int)(leftBack.getEncoder().getPosition() / 10.71);
		rightBackPositionZero = (int)(rightBack.getEncoder().getPosition() / 10.71);
	}

	//Not Field-Oriented (aka Robot-Oriented)
	public void driveCartesian(double ySpeed, double xSpeed, double zRotation) {
		robotDrive.driveCartesian(ySpeed, xSpeed, zRotation);
	}
	// Field-Oriented
	public void driveCartesian(double ySpeed, double xSpeed, double zRotation, double currentAngle) {
		robotDrive.driveCartesian(ySpeed, xSpeed, zRotation, currentAngle);
	}
	
	/** Get the encoder positions or speeds **************************************/
	public double getLeftFrontPosition() { // Position is returned in units of revolutions
		return (int)(leftFront.getEncoder().getPosition() / 10.71) - leftFrontPositionZero;
	}
	public double getRightFrontPosition() { // Position is returned in units of revolutions
		return (int)(rightFront.getEncoder().getPosition() / 10.71) - rightFrontPositionZero;
	}
	public double getLeftBackPosition() { // Position is returned in units of revolutions
		return (int)(leftBack.getEncoder().getPosition() / 10.71) - leftBackPositionZero;
	}
	public double getRightBackPosition() { // Position is returned in units of revolutions
		return (int)(rightBack.getEncoder().getPosition() / 10.71) - rightBackPositionZero;
	}
	public double getLeftFrontSpeed() { // Speed is returned in units of RPM (revolutions per minute)
		return (int)(leftFront.getEncoder().getVelocity() / 10.71);
	}
	public double getRightFrontSpeed() { // Speed is returned in units of RPM (revolutions per minute)
		return (int)(rightFront.getEncoder().getVelocity() / 10.71);
	}
	public double getLeftBackSpeed() { // Speed is returned in units of RPM (revolutions per minute)
		return (int)(leftBack.getEncoder().getVelocity() / 10.71);
	}
	public double getRightBackSpeed() { // Speed is returned in units of RPM (revolutions per minute)
		return (int)(rightBack.getEncoder().getVelocity() / 10.71);
	}

	public void setWheelSpeeds(MecanumDriveWheelSpeeds speeds) {
		final double frontLeftFeedforward = kFeedforward.calculate(speeds.frontLeftMetersPerSecond);
		final double frontRightFeedforward = kFeedforward.calculate(speeds.frontRightMetersPerSecond);
		final double backLeftFeedforward = kFeedforward.calculate(speeds.rearLeftMetersPerSecond);
		final double backRightFeedforward = kFeedforward.calculate(speeds.rearRightMetersPerSecond);

		final double frontLeftOutput =
			frontLeftPIDController.calculate(speedToMeters(getLeftFrontSpeed()), speeds.frontLeftMetersPerSecond);
		final double frontRightOutput =
			frontRightPIDController.calculate(speedToMeters(getRightFrontSpeed()), speeds.frontRightMetersPerSecond);
		final double backLeftOutput =
			backLeftPIDController.calculate(speedToMeters(getLeftBackSpeed()), speeds.rearLeftMetersPerSecond);
		final double backRightOutput =
			backRightPIDController.calculate(speedToMeters(getRightBackSpeed()), speeds.rearRightMetersPerSecond);

		leftFront.setVoltage(frontLeftOutput + frontLeftFeedforward);
		rightFront.setVoltage(frontRightOutput + frontRightFeedforward);
		leftBack.setVoltage(backLeftOutput + backLeftFeedforward);
		rightBack.setVoltage(backRightOutput + backRightFeedforward);
	}

	// Conversion Methods: Convert to Meters
	public double positionToMeters(double position) {
		return position * Math.PI * RobotMap.WHEEL_DIAMETER / 39.3700787; // Divide by 39.3700787 to convert inches to meters
	}
	public double speedToMeters(double speed) {
		return speed / 60 * Math.PI * RobotMap.WHEEL_DIAMETER / 39.3700787; // Divide by 39.3700787 to convert inches to meters
	}

	// This is only used by Drivetrain_GyroStraight()
	public double rpmToPower(double rpm, double kF) {
		//Convert rpm to counts per 100 ms
		double speed = rpm * COUNTS_PER_REVOLUTION / 600.0; // Dividing by 600 converts the units from minutes to 100s of milliseconds
		double power = kF * speed / 1023.0;
		return power;
	}
}