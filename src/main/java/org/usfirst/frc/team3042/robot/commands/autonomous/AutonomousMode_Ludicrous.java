package org.usfirst.frc.team3042.robot.commands.autonomous;

import org.usfirst.frc.team3042.robot.Robot;
import org.usfirst.frc.team3042.robot.commands.Conveyor_Advance;
import org.usfirst.frc.team3042.robot.commands.Drivetrain_GyroStraight;
import org.usfirst.frc.team3042.robot.commands.autonomous.helperCommands.Wait;
import org.usfirst.frc.team3042.robot.subsystems.Conveyor;
import org.usfirst.frc.team3042.robot.subsystems.Intake;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

/** Autonomous Mode (Ludicrous) - 10 points!!! ******************************************************
 * The fanciest autonomous routine concievably possible, this is sure to win us fame and glory! */
public class AutonomousMode_Ludicrous extends SequentialCommandGroup {

  Intake intake = Robot.intake;
  Conveyor conveyor = Robot.conveyor;

  public AutonomousMode_Ludicrous() {
    addCommands(new InstantCommand(intake::extend, intake), new InstantCommand(intake::autoSetPower, intake), // Deploy the intake and start running it
                Robot.constructTrajectoryCommand("RightTarmac_1Ball"), // Drive our trajectory to intake 1 more cargo
                new InstantCommand(conveyor::autoSetPower, conveyor), new InstantCommand(intake::stop, intake), new Wait(2), new InstantCommand(conveyor::stop, conveyor), new InstantCommand(intake::autoSetPower, intake), // Score our first 2 cargo
                new ParallelCommandGroup(Robot.constructTrajectoryCommand("Ludicrous_Mode"), new Conveyor_Advance()), // Drive our trajectory to intake 2 more cargo
                new InstantCommand(conveyor::autoSetPower, conveyor), new InstantCommand(intake::stop, intake), new Wait(2), new InstantCommand(conveyor::stop, conveyor), // Stop the intake and score our additional cargo
                new Drivetrain_GyroStraight(40, 0.5)); // This just makes it easier for the driver to spin and zero the gyro after auto :)
  }
}