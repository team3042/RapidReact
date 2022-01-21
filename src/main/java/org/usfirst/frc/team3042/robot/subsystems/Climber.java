package org.usfirst.frc.team3042.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import org.usfirst.frc.team3042.lib.Log;
import org.usfirst.frc.team3042.robot.RobotMap;

import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.util.sendable.SendableRegistry;

//import edu.wpi.first.wpilibj2.command.SubsystemBase;

/** Climber ********************************************************************
 * Subsystem for Climbing */
public class Climber extends Subsystem {
	/** Configuration Constants ***********************************************/
<<<<<<< HEAD
	private static final Log.Level LOG_LEVEL = RobotMap.LOG_CLIMBER;
	private static final int CAN_CLIMBER = RobotMap.CAN_CLIMBER;
=======
	private static final Log.Level LOG_LEVEL = RobotMap.LOG_INTAKE;
	private static final int CAN_RIGHT_CLIMBER = RobotMap.CAN_RIGHT_CLIMBER;
	private static final int CAN_LEFT_CLIMBER = RobotMap.CAN_LEFT_CLIMBER;
>>>>>>> 6c61e581ed4d876ef475f9ed32aef05981d84771
	private static final boolean REVERSE_MOTOR = RobotMap.REVERSE_CLIMBER;
	private static final NeutralMode BRAKE_MODE = RobotMap.CLIMBER_BRAKE_MODE;

	/** Instance Variables ****************************************************/
	Log log = new Log(LOG_LEVEL, SendableRegistry.getName(this));
	TalonSRX rightMotor = new TalonSRX(CAN_RIGHT_CLIMBER);
	TalonSRX leftMotor = new TalonSRX(CAN_LEFT_CLIMBER);

	/** Climber ****************************************************************/
	public Climber() {
		log.add("Constructor", LOG_LEVEL);
		initMotor(rightMotor, REVERSE_MOTOR);
		initMotor(leftMotor, REVERSE_MOTOR);
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
	public void stop() {
		setPower(0.0);
	}
	private double safetyCheck(double power) {
		power = Math.min(1.0, power);
		power = Math.max(-1.0, power);
		return power;
	}
	
	/** initDefaultCommand ****************************************************
	 * Set the default command for the subsystem. */
		public void initDefaultCommand() {
		setDefaultCommand(null);
	}
}