package org.usfirst.frc.team3042.robot.triggers;

import org.usfirst.frc.team3042.lib.Log;
import org.usfirst.frc.team3042.robot.RobotMap;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/** POVButton *****************************************************************
 * Creates a trigger from a POV button. */
public class POVButton extends Trigger {
	/** Configuration Constants ***********************************************/
	public static final Log.Level LOG_LEVEL = RobotMap.LOG_POV_BUTTON;
	
	/** POV Direction Angles **************************************************/
	public static final int UP = 0;
	public static final int UP_RIGHT = 45;
	public static final int RIGHT = 90;
	public static final int DOWN_RIGHT = 135;
	public static final int DOWN = 180;
	public static final int DOWN_LEFT = 225;
	public static final int LEFT = 270;
	public static final int UP_LEFT = 315;

	/** Instance Variables ****************************************************/
	Log log = new Log(LOG_LEVEL, "POV Button");
	int direction;
	Joystick gamepad;
	
	/** POVBUtton *************************************************************
	 * Joystick	gamepad		The gamepad to monitor
	 * int 		direction	The angle, in degrees, of the POV hat that 
	 * 						triggers action */
	public POVButton (Joystick gamepad, int direction) {
		log.add("Constructor "+direction, Log.Level.TRACE);
		
		this.gamepad = gamepad;
		this.direction = direction;
	}
	
	/** get *******************************************************************
	 * Returns true if the POV hat is pressed in the specified direction. */
    public boolean get() {
        return gamepad.getPOV() == direction;
    }
}