package org.usfirst.frc.team3042.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import org.usfirst.frc.team3042.lib.Log;
import org.usfirst.frc.team3042.robot.RobotMap;

import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/** Conveyor ********************************************************************
 * Subsystem for moving balls through the body of the robot */
public class Conveyor extends SubsystemBase {
	/** Configuration Constants ***********************************************/
	private static final Log.Level LOG_LEVEL = RobotMap.LOG_CONVEYOR;
  	private static final int CAN_TOP_CONVEYOR = RobotMap.CAN_TOP_CONVEYOR;
	private static final int CAN_LEFT_CONVEYOR = RobotMap.CAN_LEFT_CONVEYOR;
	private static final int CAN_RIGHT_CONVEYOR = RobotMap.CAN_RIGHT_CONVEYOR;
	private static final boolean REVERSE_TOP_CONVEYOR = RobotMap.REVERSE_TOP_CONVEYOR;
	private static final boolean REVERSE_LEFT_CONVEYOR = RobotMap.REVERSE_LEFT_CONVEYOR;
	private static final boolean REVERSE_RIGHT_CONVEYOR = RobotMap.REVERSE_RIGHT_CONVEYOR;
	private static final NeutralMode BRAKE_MODE = RobotMap.CONVEYOR_BRAKE_MODE;
	private static final int DIO_LIMITSWITCH = RobotMap.DIO_LIMITSWITCH_CONVEYOR;
	private static final double POWER = RobotMap.CONVEYOR_POWER;

  	/** Instance Variables ****************************************************/
	Log log = new Log(LOG_LEVEL, SendableRegistry.getName(this));
	TalonSRX topMotor = new TalonSRX(CAN_TOP_CONVEYOR);
	TalonSRX leftMotor = new TalonSRX(CAN_LEFT_CONVEYOR);
	TalonSRX rightMotor = new TalonSRX(CAN_RIGHT_CONVEYOR);
	DigitalInput limit = new DigitalInput(DIO_LIMITSWITCH);

  	/** Conveyor ****************************************************************/
	public Conveyor() {
	  	log.add("Constructor", LOG_LEVEL);
		initMotor(topMotor, REVERSE_TOP_CONVEYOR);
		initMotor(leftMotor, REVERSE_LEFT_CONVEYOR);
		initMotor(rightMotor, REVERSE_RIGHT_CONVEYOR);
  	}

  private void initMotor(TalonSRX motor, boolean reverse) {
		motor.setNeutralMode(BRAKE_MODE);
		motor.setInverted(reverse); // affects percent Vbus mode
  	}
  
 	 /** Methods for setting the motor in Percent Vbus mode **********************/
	public void setPower(double Power) {
		Power = safetyCheck(Power);
		topMotor.set(ControlMode.PercentOutput, Power);
		leftMotor.set(ControlMode.PercentOutput, Power);
		rightMotor.set(ControlMode.PercentOutput, Power);		
	}
	public void autoSetPower() {
		double Power = safetyCheck(POWER);
		topMotor.set(ControlMode.PercentOutput, Power);
		leftMotor.set(ControlMode.PercentOutput, Power);
		rightMotor.set(ControlMode.PercentOutput, Power);		
	}
	public void stop() {
		setPower(0.0);
	}
	private double safetyCheck(double power) {
		power = Math.min(1.0, power);
		power = Math.max(-1.0, power);
		return power;
	}
	
	public boolean getLimitStatus() {
		return !limit.get();
	}
	
	/** initDefaultCommand ****************************************************
	 * Set the default command for the subsystem. */
	public void initDefaultCommand() {
		setDefaultCommand(null);
	}
}