package org.usfirst.frc.team3042.robot.commands.autonomous;

import org.usfirst.frc.team3042.robot.commands.Drivetrain_Trajectory;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

/** Autonomous Mode Test Curve 2 ******************************************************
 * This tests our curve 2 path trajectory */
public class AutonomousMode_TestCurvePath2 extends SequentialCommandGroup {

  public AutonomousMode_TestCurvePath2() {
    addCommands(new Drivetrain_Trajectory("Basic_Curve_Path2"));
  }
}