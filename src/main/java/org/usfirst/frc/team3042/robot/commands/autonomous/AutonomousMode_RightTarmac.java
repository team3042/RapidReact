package org.usfirst.frc.team3042.robot.commands.autonomous;

import org.usfirst.frc.team3042.robot.Robot;
import org.usfirst.frc.team3042.robot.commands.Conveyor_Advance;
import org.usfirst.frc.team3042.robot.commands.Conveyor_Run;
import org.usfirst.frc.team3042.robot.commands.Drivetrain_GyroStraight;
import org.usfirst.frc.team3042.robot.commands.Intake_Intake;
import org.usfirst.frc.team3042.robot.commands.autonomous.helperCommands.Wait;
import org.usfirst.frc.team3042.robot.subsystems.Intake;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

/** Autonomous Mode (Right Tarmac) - 8 points!! ******************************************************
 * A fancier autonomous routine for scoring 3 cargo when we start in the right tarmac */
public class AutonomousMode_RightTarmac extends SequentialCommandGroup {

  Intake intake = Robot.intake;

  public AutonomousMode_RightTarmac() {
    addCommands(new Conveyor_Run(1), new Wait(2), new Conveyor_Run(0), // Run the conveyor for a specified number of seconds
                new InstantCommand(intake::extend, intake), new Intake_Intake(1), // Deploy the intake and start running it
                new ParallelCommandGroup(Robot.constructTrajectoryCommand("Right_Tarmac"), new Conveyor_Advance()), // Drive our trajectory to intake 2 more cargo
                new Conveyor_Run(1), new Intake_Intake(0), new Wait(2), new Conveyor_Run(0), // Stop the intake and score our additional cargo
                new Drivetrain_GyroStraight(40, 0.5)); // This just makes it easier for the driver to spin and zero the gyro after auto :)
  }
}