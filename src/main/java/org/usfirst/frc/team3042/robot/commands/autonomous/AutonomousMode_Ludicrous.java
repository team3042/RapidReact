package org.usfirst.frc.team3042.robot.commands.autonomous;

import org.usfirst.frc.team3042.robot.Robot;
import org.usfirst.frc.team3042.robot.RobotMap;
import org.usfirst.frc.team3042.robot.commands.Conveyor_Advance;
import org.usfirst.frc.team3042.robot.commands.Drivetrain_GyroStraight;
import org.usfirst.frc.team3042.robot.commands.autonomous.helperCommands.Wait;
import org.usfirst.frc.team3042.robot.subsystems.Conveyor;
import org.usfirst.frc.team3042.robot.subsystems.Intake;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

/** Autonomous Mode (Ludicrous) - 10 points!!! ******************************************************
 * This fancy autonomous routine is sure to win us fame and glory! :) */
public class AutonomousMode_Ludicrous extends SequentialCommandGroup {

  Intake intake = Robot.intake;
  Conveyor conveyor = Robot.conveyor;

  public AutonomousMode_Ludicrous() {
    addCommands(new InstantCommand(intake::extend, intake), new Wait(.1), // Deploy the intake
                new InstantCommand(intake::autoSetPower, intake), // Start running Intake
                Robot.constructTrajectoryCommand("RightTarmac_1Ball", 5.5, RobotMap.ACCELERATION_MAX_MPS), // Drive our trajectory path to intake more cargo
                new InstantCommand(conveyor::autoSetPower, conveyor), new InstantCommand(intake::stop, intake), new Wait(0.75), new InstantCommand(conveyor::stop, conveyor), new InstantCommand(intake::autoSetPower, intake), // Score our first 2 cargo
                new ParallelCommandGroup(Robot.constructTrajectoryCommand("Ludicrous_Mode", 5.5, RobotMap.ACCELERATION_MAX_MPS), new Conveyor_Advance()), // Drive our trajectory to intake 2 more cargo
                new InstantCommand(conveyor::autoSetPower, conveyor), new InstantCommand(intake::stop, intake), new Wait(0.75), new InstantCommand(conveyor::stop, conveyor), // Stop the intake and score our additional cargo
                new Drivetrain_GyroStraight(40, 0.8)); // This just makes it easier for the driver to spin and zero the gyro after auto :)
  }
}