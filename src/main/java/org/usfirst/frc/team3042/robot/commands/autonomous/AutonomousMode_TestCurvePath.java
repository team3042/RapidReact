package org.usfirst.frc.team3042.robot.commands.autonomous;

import org.usfirst.frc.team3042.robot.commands.Drivetrain_Trajectory;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

/** Autonomous Mode Test Curve ******************************************************
 * This tests our curve  path trajectory */
public class AutonomousMode_TestCurvePath extends SequentialCommandGroup {

  public AutonomousMode_TestCurvePath() {
    addCommands(new Drivetrain_Trajectory("Basic_Curve_Path"));
  }
}