package org.usfirst.frc.team3042.robot.commands.autonomous;

import org.usfirst.frc.team3042.robot.commands.Conveyor_Run;
import org.usfirst.frc.team3042.robot.commands.Drivetrain_Trajectory;
import org.usfirst.frc.team3042.robot.commands.Intake_Intake;
import org.usfirst.frc.team3042.robot.commands.Intake_Toggle;
import org.usfirst.frc.team3042.robot.commands.autonomous.helperCommands.Wait;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

/** Autonomous Mode (OP) - 10 points!!! ******************************************************
 * The fanciest autonomous routine concievably possible, this is sure to win us fame and glory! */
public class AutonomousMode_OP extends SequentialCommandGroup {

  public AutonomousMode_OP() {
    addCommands(new Conveyor_Run(1), new Wait(1.75), new Conveyor_Run(0), // Run the conveyor for a specified number of seconds
                new Intake_Toggle(), new Intake_Intake(1), // Deploy the intake and start running it
                new ParallelCommandGroup(new Drivetrain_Trajectory("Right_Tarmac"), // Drive our trajectory to intake 2 more cargo
                                         new ParallelCommandGroup(new Wait(1), new Conveyor_Run(0.5), new Wait(1), new Conveyor_Run(0))), //TODO: Tune these wait times!
                new Conveyor_Run(1), new Intake_Intake(0), new Wait(1.75), new Conveyor_Run(0), new Intake_Intake(1), // Stop the intake and score our additional cargo
                new Drivetrain_Trajectory("Player_Station_Cargo"), // Drive our trajectory to intake 1 more cargo
                new Conveyor_Run(1), new Intake_Intake(0)); // Stop the intake and score our final cargo!
  }
}