package org.usfirst.frc.team3042.robot.commands.autonomous;

import org.usfirst.frc.team3042.robot.commands.drivetrain.Drivetrain_GyroStraight;

import edu.wpi.first.wpilibj.command.CommandGroup;

/** Autonomous Mode Default ******************************************************
 * Our default autonomous mode that drives forwards off the initiation line and then shoots the three pre-loaded power cells */
public class AutonomousMode_Default extends CommandGroup {

  public AutonomousMode_Default() {
    addSequential(new Drivetrain_GyroStraight(18.0, 120.0)); //Drive forwards off the initiation line
  }
}