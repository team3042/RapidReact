package org.usfirst.frc.team3042.robot.commands.autonomous;

import org.usfirst.frc.team3042.robot.commands.Drivetrain_Trajectory;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

/** Autonomous Mode Test Strafe ******************************************************
 * This tests our strafe path trajectory */
public class AutonomousMode_TestStrafePath extends SequentialCommandGroup {

  public AutonomousMode_TestStrafePath() {
    addCommands(new Drivetrain_Trajectory("Basic_Strafe_Path"));
  }
}