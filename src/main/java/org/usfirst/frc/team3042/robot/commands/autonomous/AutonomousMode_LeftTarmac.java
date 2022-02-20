package org.usfirst.frc.team3042.robot.commands.autonomous;

import org.usfirst.frc.team3042.robot.Robot;
import org.usfirst.frc.team3042.robot.commands.Conveyor_Run;
import org.usfirst.frc.team3042.robot.commands.Intake_Intake;
import org.usfirst.frc.team3042.robot.commands.Intake_Toggle;
import org.usfirst.frc.team3042.robot.commands.autonomous.helperCommands.Wait;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

/** Autonomous Mode (Left Tarmac) - 6 points! ******************************************************
 * A fancier autonomous routine for scoring 2 cargo when we start in the left tarmac */
public class AutonomousMode_LeftTarmac extends SequentialCommandGroup {

  public AutonomousMode_LeftTarmac() {
    addCommands(new Conveyor_Run(1), new Wait(2), new Conveyor_Run(0), // Run the conveyor for a specified number of seconds
                new Intake_Toggle(), new Intake_Intake(1), // Deploy the intake and start running it
                Robot.constructTrajectoryCommand("Left_Tarmac"), // Drive our trajectory to intake 1 more cargo
                new Conveyor_Run(1), new Intake_Intake(0)); // Stop the intake and score our additional cargo
  }
}