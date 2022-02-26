package org.usfirst.frc.team3042.robot.commands.autonomous;

import org.usfirst.frc.team3042.robot.Robot;
import org.usfirst.frc.team3042.robot.commands.Conveyor_Advance;
import org.usfirst.frc.team3042.robot.commands.Conveyor_Run;
import org.usfirst.frc.team3042.robot.commands.Intake_Intake;
import org.usfirst.frc.team3042.robot.commands.Intake_Toggle;
import org.usfirst.frc.team3042.robot.commands.autonomous.helperCommands.Wait;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

/** Autonomous Mode (Right Tarmac) - 8 points!! ******************************************************
 * A fancier autonomous routine for scoring 3 cargo when we start in the right tarmac */
public class AutonomousMode_RightTarmac extends SequentialCommandGroup {

  public AutonomousMode_RightTarmac() {
    addCommands(new Conveyor_Run(1), new Wait(2), new Conveyor_Run(0), // Run the conveyor for a specified number of seconds
                new Intake_Toggle(), new Intake_Intake(1), // Deploy the intake and start running it
                new ParallelCommandGroup(Robot.constructTrajectoryCommand("Ludicrous_Mode"), new Conveyor_Advance()), // Drive our trajectory to intake 2 more cargo
                new Conveyor_Run(1), new Intake_Intake(0)); // Stop the intake and score our additional cargo
  }
}