package org.usfirst.frc.team3042.robot.commands.autonomous;

import org.usfirst.frc.team3042.robot.Robot;
import org.usfirst.frc.team3042.robot.commands.Conveyor_Run;
import org.usfirst.frc.team3042.robot.commands.Intake_Intake;
import org.usfirst.frc.team3042.robot.commands.Intake_Toggle;
import org.usfirst.frc.team3042.robot.commands.autonomous.helperCommands.Wait;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

/** Autonomous Mode (Ludicrous) - 10 points!!! ******************************************************
 * The fanciest autonomous routine concievably possible, this is sure to win us fame and glory! */
public class AutonomousMode_Ludicrous extends SequentialCommandGroup {

  public AutonomousMode_Ludicrous() {
    addCommands(new Intake_Toggle(), new Intake_Intake(1), // Deploy the intake and start running it
                Robot.constructTrajectoryCommand("RightTarmac_1Ball"), // Drive our trajectory to intake 1 more cargo
                new Conveyor_Run(1), new Intake_Intake(0), new Wait(2), new Conveyor_Run(0), new Intake_Intake(1)); // Score our first 2 cargo
                new ParallelCommandGroup(Robot.constructTrajectoryCommand("Ludicrous_Mode"), // Drive our trajectory to intake 2 more cargo
                                         new SequentialCommandGroup(new Wait(1), new Conveyor_Run(0.5), new Wait(1), new Conveyor_Run(0)), //TODO: Tune these wait times to bring the first ball into the conveyor!
                new Conveyor_Run(1), new Intake_Intake(0)); // Stop the intake and score our additional cargo
  }
}