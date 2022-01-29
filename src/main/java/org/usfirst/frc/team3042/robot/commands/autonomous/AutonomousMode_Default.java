package org.usfirst.frc.team3042.robot.commands.autonomous;

import org.usfirst.frc.team3042.robot.commands.Drivetrain_GyroStraight;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

/** Autonomous Mode Default ******************************************************
 * A default autonomous mode template that simply drives forwards */
public class AutonomousMode_Default extends SequentialCommandGroup {

  public AutonomousMode_Default() {
    addCommands(new Drivetrain_GyroStraight(18.0, 120.0)); //Drive forwards
  }
}