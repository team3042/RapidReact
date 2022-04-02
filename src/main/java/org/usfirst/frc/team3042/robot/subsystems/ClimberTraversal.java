package org.usfirst.frc.team3042.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import org.usfirst.frc.team3042.lib.Log;
import org.usfirst.frc.team3042.robot.RobotMap;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.util.sendable.SendableRegistry;

/** ClimberTraversal ********************************************************************
 * Subsystem for Climber Traversal */
public class ClimberTraversal extends SubsystemBase {
	/** Configuration Constants ***********************************************/
	private static final Log.Level LOG_LEVEL = RobotMap.LOG_CLIMBER;
	private static final int CAN_TRAVERSAL_MOTOR = RobotMap.CAN_TRAVERSAL_MOTOR;
	private static final boolean REVERSE_TRAVERSAL_CLIMBER = RobotMap.REVERSE_TRAVERSAL_CLIMBER;
	private static final IdleMode TRAVERSAL_CLIMBER_BRAKE_MODE = RobotMap.TRAVERSAL_CLIMBER_BRAKE_MODE;

	/** Instance Variables ****************************************************/
	Log log = new Log(LOG_LEVEL, SendableRegistry.getName(this));
	CANSparkMax winch = new CANSparkMax(CAN_TRAVERSAL_MOTOR, MotorType.kBrushless);	
	boolean isRetracted = true;

	/** ClimberTraversal ****************************************************************/
	public ClimberTraversal() {
		log.add("Constructor", LOG_LEVEL);
		winch.restoreFactoryDefaults();
		initMotor(winch, REVERSE_TRAVERSAL_CLIMBER);
		
		resetEncoder();
	}
	private void initMotor(CANSparkMax motor, boolean reverse) {
		motor.setIdleMode(TRAVERSAL_CLIMBER_BRAKE_MODE);
		motor.setInverted(reverse);
	}
  
	/** Methods for setting the motor in Percent Vbus mode **********************/
	public void setPower(double Power) {
		Power = safetyCheck(Power);
		winch.set(Power);
	}	
	private double safetyCheck(double power) {
		power = Math.min(1.0, power);
		power = Math.max(-1.0, power);
		return power;
	}

	// Stops the winch
	public void stop() {
		setPower(0.0);
	}

	public void toggle() {
		if (isRetracted == true) {
			isRetracted = false;
		}
		else if (isRetracted == false) {
			isRetracted = true;
		}
	}
	public boolean isRetracted() {
		return isRetracted;
	}

	/** Get the encoder positions or speeds **************************************/
	public void resetEncoder() {
		winch.getEncoder().setPosition(0);
	}
	public double getWinchPosition() { // Position is returned in units of revolutions
		return winch.getEncoder().getPosition();
	}
	
	/** initDefaultCommand ****************************************************
	 * Set the default command for the subsystem. */
		public void initDefaultCommand() {
		setDefaultCommand(null);
	}
}