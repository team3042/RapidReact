package org.usfirst.frc.team3042.robot.commands.autonomous;

import org.usfirst.frc.team3042.robot.Robot;
import org.usfirst.frc.team3042.robot.commands.Conveyor_Run;
import org.usfirst.frc.team3042.robot.commands.Drivetrain_GyroStraight;
import org.usfirst.frc.team3042.robot.commands.autonomous.helperCommands.Wait;
import org.usfirst.frc.team3042.robot.subsystems.Intake;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

/** Autonomous Mode (Default) - 4 points ******************************************************
 * This is our most basic autonomous routine for scoring the starting cargo */
public class AutonomousMode_Default extends SequentialCommandGroup {

  Intake intake = Robot.intake;

  public AutonomousMode_Default() {
    addCommands(new Conveyor_Run(1), new Wait(2), new Conveyor_Run(0), // Run the conveyor for a specified number of seconds
                new Drivetrain_GyroStraight(80, 0.5), new InstantCommand(intake::extend, intake)); // Drive out of the tarmac and deploy the intake!
  }
}