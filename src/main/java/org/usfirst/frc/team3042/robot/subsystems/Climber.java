package org.usfirst.frc.team3042.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import org.usfirst.frc.team3042.lib.Log;
import org.usfirst.frc.team3042.robot.RobotMap;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.util.sendable.SendableRegistry;

/** Climber ********************************************************************
 * Subsystem for Climbing */
public class Climber extends SubsystemBase {
	/** Configuration Constants ***********************************************/
	private static final Log.Level LOG_LEVEL = RobotMap.LOG_CLIMBER;
	private static final int CAN_RIGHT_CLIMBER = RobotMap.CAN_RIGHT_CLIMBER;
	private static final int CAN_LEFT_CLIMBER = RobotMap.CAN_LEFT_CLIMBER;
	private static final int ID = RobotMap.CLIMBER_SOLENOID;
	private static final boolean REVERSE_RIGHT_MOTOR = RobotMap.REVERSE_RIGHT_CLIMBER;
	private static final boolean REVERSE_LEFT_MOTOR = RobotMap.REVERSE_LEFT_CLIMBER;
	private static final NeutralMode BRAKE_MODE = RobotMap.CLIMBER_BRAKE_MODE;
	private static final int DIO_LIMITSWITCH_RIGHT = RobotMap.DIO_LIMITSWITCH_CLIMBER_RIGHT;
	private static final int DIO_LIMITSWITCH_LEFT = RobotMap.DIO_LIMITSWITCH_CLIMBER_LEFT;

	/** Instance Variables ****************************************************/
	Log log = new Log(LOG_LEVEL, SendableRegistry.getName(this));
	TalonSRX rightMotor = new TalonSRX(CAN_RIGHT_CLIMBER);
	TalonSRX leftMotor = new TalonSRX(CAN_LEFT_CLIMBER);
	Solenoid ClimberSolenoid = new Solenoid(PneumaticsModuleType.CTREPCM, ID);
	DigitalInput limitRight = new DigitalInput(DIO_LIMITSWITCH_RIGHT);
	DigitalInput limitLeft = new DigitalInput(DIO_LIMITSWITCH_LEFT);
	public boolean isRetracted = true;

	/** Climber ****************************************************************/
	public Climber() {
		log.add("Constructor", LOG_LEVEL);
		initMotor(rightMotor, REVERSE_RIGHT_MOTOR);
		initMotor(leftMotor, REVERSE_LEFT_MOTOR);
	}

	private void initMotor(TalonSRX motor, boolean reverse) {
		motor.setNeutralMode(BRAKE_MODE);
		motor.setInverted(reverse); // affects percent Vbus mode
  	}
  
	/** Methods for setting the motor in Percent Vbus mode **********************/
	public void setPower(double Power) {
		Power = safetyCheck(Power);
		rightMotor.set(ControlMode.PercentOutput, Power);
		leftMotor.set(ControlMode.PercentOutput, Power);		
	}
	public void setLeftPower(double Power) {
		Power = safetyCheck(Power);
		leftMotor.set(ControlMode.PercentOutput, Power);		
	}
	public void setRightPower(double Power) {
		Power = safetyCheck(Power);
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

	public boolean getLeftLimitStatus() {
		return !limitLeft.get();
	}
	public boolean getRightLimitStatus() {
		return !limitRight.get();
	}
	
	// Commands for extending/retracting the climber
	public void extend() {
		ClimberSolenoid.set(false);
		isRetracted = false;
	}
	public void retract() {
		ClimberSolenoid.set(true);
		isRetracted = true;
	}
	public void toggle() {
		if (isRetracted == true) {
			this.extend();
		}
		else {
			this.retract();
		}
	}
	
	/** initDefaultCommand ****************************************************
	 * Set the default command for the subsystem. */
		public void initDefaultCommand() {
		setDefaultCommand(null);
	}
}