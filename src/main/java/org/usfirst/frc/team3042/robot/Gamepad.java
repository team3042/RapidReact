package org.usfirst.frc.team3042.robot;

import org.usfirst.frc.team3042.robot.triggers.AxisTrigger;
import org.usfirst.frc.team3042.robot.triggers.POVButton;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.button.Button;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class Gamepad extends Joystick{
	/** Gamepad Button and Axis Mapping ***************************************/
	public static final int BUTTON_A = 1;
	public static final int BUTTON_B = 2;
	public static final int BUTTON_X = 3;
	public static final int BUTTON_Y = 4;
	public static final int BUTTON_LB = 5;
	public static final int BUTTON_RB = 6;
	public static final int BUTTON_BACK = 7;
	public static final int BUTTON_START = 8;
	public static final int BUTTON_LEFT_JOY = 9;
	public static final int BUTTON_RIGHT_JOY = 10;

	public static final int LEFT_JOY_X_AXIS = 0;
	public static final int LEFT_JOY_Y_AXIS = 1;
	public static final int LEFT_TRIGGER = 2;
	public static final int RIGHT_TRIGGER = 3;
	public static final int RIGHT_JOY_X_AXIS = 4;
	public static final int RIGHT_JOY_Y_AXIS = 5;
	
	/** Axis Mapping for a single joystick ************************************/
	public static final int JOY_X_AXIS = 0;
	public static final int JOY_Y_AXIS = 1;
	public static final int JOY_Z_AXIS = 2;
	
	/** Button Declarations ***************************************************/
	Button A = new JoystickButton(this, BUTTON_A);
	Button B = new JoystickButton(this, BUTTON_B);
	Button X = new JoystickButton(this, BUTTON_X);
	Button Y = new JoystickButton(this, BUTTON_Y);
	Button LB = new JoystickButton(this, BUTTON_LB);
	Button RB = new JoystickButton(this, BUTTON_RB);
	Button Back = new JoystickButton(this, BUTTON_BACK);
	Button Start = new JoystickButton(this, BUTTON_START);
	Button LeftJoy = new JoystickButton(this, BUTTON_LEFT_JOY);
	Button RightJoy = new JoystickButton(this, BUTTON_RIGHT_JOY);
	
	/** Trigger Declarations **************************************************/
	Trigger LeftJoyLeft = new AxisTrigger(this, LEFT_JOY_X_AXIS, AxisTrigger.Direction.LEFT);
	Trigger LeftJoyRight = new AxisTrigger(this, LEFT_JOY_X_AXIS, AxisTrigger.Direction.RIGHT);
	Trigger LeftJoyUp = new AxisTrigger(this, LEFT_JOY_Y_AXIS, AxisTrigger.Direction.UP);
	Trigger LeftJoyDown = new AxisTrigger(this, LEFT_JOY_Y_AXIS, AxisTrigger.Direction.DOWN);
	
	Trigger LT = new AxisTrigger(this, LEFT_TRIGGER);
	Trigger RT = new AxisTrigger(this, RIGHT_TRIGGER);
	
	Trigger RightJoyLeft = new AxisTrigger(this, RIGHT_JOY_X_AXIS, AxisTrigger.Direction.LEFT);
	Trigger RightJoyRight = new AxisTrigger(this, RIGHT_JOY_X_AXIS, AxisTrigger.Direction.RIGHT);
	Trigger RightJoyUp = new AxisTrigger(this, RIGHT_JOY_Y_AXIS, AxisTrigger.Direction.UP);
	Trigger RightJoyDown = new AxisTrigger(this, RIGHT_JOY_Y_AXIS, AxisTrigger.Direction.DOWN);
	
	Trigger POVUp = new POVButton(this, POVButton.UP);
	Trigger POVDown = new POVButton(this, POVButton.DOWN);
	Trigger POVLeft = new POVButton(this, POVButton.LEFT);
	Trigger POVRight = new POVButton(this, POVButton.RIGHT);
	
	/** Joystick equivalent buttons *******************************************/
	Button button1 = A;
	Button button2 = B;
	Button button3 = X;
	Button button4 = Y;
	Button button5 = LB;
	Button button6 = RB;
	Button button7 = Back;
	Button button8 = Start;
	Button button9 = LeftJoy;
	Button button10 = RightJoy;
	
	public Gamepad (int port) {
		super(port);
	}
}