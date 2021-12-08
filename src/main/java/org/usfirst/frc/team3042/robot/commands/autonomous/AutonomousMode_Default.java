package org.usfirst.frc.team3042.robot.commands.autonomous;

import org.usfirst.frc.team3042.robot.commands.drivetrain.Drivetrain_GyroStraight;

import edu.wpi.first.wpilibj.command.CommandGroup;

/** Autonomous Mode Default ******************************************************
 * A default autonomous mode template that simply drives forwards */
public class AutonomousMode_Default extends CommandGroup {

  public AutonomousMode_Default() {
    addSequential(new Drivetrain_GyroStraight(18.0, 120.0)); //Drive forwards
  }
}