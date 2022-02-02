package org.usfirst.frc.team3042.robot.triggers;

import org.usfirst.frc.team3042.lib.Log;
import org.usfirst.frc.team3042.robot.RobotMap;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/** GamepadTrigger ************************************************************
 * A object that monitors gamepad triggers */
public class AxisTrigger extends Trigger {
	/** Configuration Constants ***********************************************/
	public static final Log.Level LOG_LEVEL = RobotMap.LOG_AXIS_TRIGGER;
	public static enum Direction{UP, DOWN, LEFT, RIGHT;}

	/** Instance Variables ****************************************************/
	Log log = new Log(LOG_LEVEL, "Axis Trigger");
	Joystick gamepad;
	int axis;
	Direction direction;
	Boolean reset = false;
	
	/** GamepadTrigger ******************************************************** 
	 * Short version assumes positive values, which is equivalent to the DOWN 
	 *  or LEFT direction. Useful for gamepad triggers.
	 * 
	 * Joystick		gamepad		The controller to monitor for a trigger
	 * int			axis		The joystick axis to monitor
	 * DIRECTION	direction	The axis direction on which to trigger. */
	public AxisTrigger(Joystick joystick, int axis){
		this(joystick, axis, Direction.DOWN);
	}
	public AxisTrigger(Joystick gamepad, int axis, Direction direction) {
		log.add("Constructor "+axis+" "+direction, Log.Level.TRACE);
		this.gamepad = gamepad;
		this.axis = axis;
		this.direction = direction;
	}
	
	/** get *******************************************************************
	 * Return if the axis has been pushed to trigger the command. */
    public boolean get() {
    	reset = reset || (Math.abs(gamepad.getRawAxis(axis)) < 0.25);

    	boolean triggered;
    	if ( (direction == Direction.UP) || (direction == Direction.LEFT) ){
    		triggered = gamepad.getRawAxis(axis) < -.5;
    	}
    	else {
    		triggered = gamepad.getRawAxis(axis) > .5;
    	}
    	return triggered && reset;
    }
}