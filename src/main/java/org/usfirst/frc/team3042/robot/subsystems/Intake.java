package org.usfirst.frc.team3042.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import org.usfirst.frc.team3042.lib.Log;
import org.usfirst.frc.team3042.robot.RobotMap;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.util.sendable.SendableRegistry;

/** Intake ********************************************************************
 * Subsystem for the Intake */
public class Intake extends SubsystemBase {
	/** Configuration Constants ***********************************************/
	private static final Log.Level LOG_LEVEL = RobotMap.LOG_INTAKE;
	private static final int CAN_INTAKE = RobotMap.CAN_INTAKE;
	private static final int RIGHT_ID = RobotMap.RIGHT_INTAKE_SOLENOID;
	private static final int LEFT_ID = RobotMap.LEFT_INTAKE_SOLENOID;
	private static final boolean REVERSE_MOTOR = RobotMap.REVERSE_INTAKE;
	private static final NeutralMode BRAKE_MODE = RobotMap.INTAKE_BRAKE_MODE;

	/** Instance Variables ****************************************************/
	Log log = new Log(LOG_LEVEL, SendableRegistry.getName(this));
	TalonSRX motor = new TalonSRX(CAN_INTAKE);
	//Solenoid rightIntakeSolenoid = new Solenoid(PneumaticsModuleType.CTREPCM, RIGHT_ID); //TODO: Uncomment this AFTER our pneumatics are wired up
	//Solenoid leftIntakeSolenoid = new Solenoid(PneumaticsModuleType.CTREPCM, LEFT_ID); //TODO: Uncomment this AFTER our pneumatics are wired up

	/** Intake ****************************************************************/
	public Intake() {
		log.add("Constructor", LOG_LEVEL);
		initMotor(motor, REVERSE_MOTOR);
	}

	private void initMotor(TalonSRX motor, boolean reverse) {
		motor.setNeutralMode(BRAKE_MODE);
		motor.setInverted(reverse); // affects percent Vbus mode
  	}
  
    /** Methods for setting the motor in Percent Vbus mode **********************/
	public void setPower(double Power) {
		Power = safetyCheck(Power);
		motor.set(ControlMode.PercentOutput, Power);		
	}
	public void stop() {
		setPower(0.0);
	}
	private double safetyCheck(double power) {
		power = Math.min(1.0, power);
		power = Math.max(-1.0, power);
		return power;
	}

	// Commands for extending/retracting the intake
	public void extend() {
		//rightIntakeSolenoid.set(true); //TODO: Uncomment this AFTER our pneumatics are wired up
		//leftIntakeSolenoid.set(true); //TODO: Uncomment this AFTER our pneumatics are wired up
	}
	public void retract() {
		//rightIntakeSolenoid.set(false); //TODO: Uncomment this AFTER our pneumatics are wired up
		//leftIntakeSolenoid.set(false); //TODO: Uncomment this AFTER our pneumatics are wired up
	}
	
	/** initDefaultCommand ****************************************************
	 * Set the default command for the subsystem. */
	public void initDefaultCommand() {
		setDefaultCommand(null);
	}
}