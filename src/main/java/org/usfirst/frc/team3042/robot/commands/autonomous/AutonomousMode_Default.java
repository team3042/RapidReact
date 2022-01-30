package org.usfirst.frc.team3042.robot.commands.autonomous;

import org.usfirst.frc.team3042.robot.commands.Drivetrain_Trajectory;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

/** Autonomous Mode Default ******************************************************
 * This will be our default autonomous routine for tournaments!
 * We'll want to score our starting cargo, then deploy the intake, start running the intake, and drive our trajectory, then stop the intake and score the additional two cargo */
public class AutonomousMode_Default extends SequentialCommandGroup {

  public AutonomousMode_Default() {
    //TODO: Write this command group! You'll need to list the commands we want in order within the addCommands() method below. 
    //Hint: You might want to use our Wait() command to stop the conveyor when scoring. Another Hint: You can nest a ParallelCommand Group within this list of commands!
    addCommands(new Drivetrain_Trajectory("pathplanner/Bottom2_Path.path") /* List commands sequentially in here */);
  }
}