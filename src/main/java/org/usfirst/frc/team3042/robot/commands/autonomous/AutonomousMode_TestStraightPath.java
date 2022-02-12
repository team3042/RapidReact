package org.usfirst.frc.team3042.robot.commands.autonomous;

import org.usfirst.frc.team3042.robot.commands.Drivetrain_Trajectory;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

/** Autonomous Mode Test Straight ******************************************************
 * This tests our straight path trajectory */
public class AutonomousMode_TestStraightPath extends SequentialCommandGroup {

  public AutonomousMode_TestStraightPath() {
    addCommands(new Drivetrain_Trajectory("Basic_Straight_Line_Path"));
  }
}