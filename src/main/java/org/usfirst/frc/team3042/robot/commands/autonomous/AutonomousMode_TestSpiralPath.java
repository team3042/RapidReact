package org.usfirst.frc.team3042.robot.commands.autonomous;

import org.usfirst.frc.team3042.robot.commands.Drivetrain_Trajectory;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

/** Autonomous Mode Test Spiral ******************************************************
 * This tests our spiral path trajectory */
public class AutonomousMode_TestSpiralPath extends SequentialCommandGroup {

  public AutonomousMode_TestSpiralPath() {
    addCommands(new Drivetrain_Trajectory("Basic_Spiral_Path"));
  }
}