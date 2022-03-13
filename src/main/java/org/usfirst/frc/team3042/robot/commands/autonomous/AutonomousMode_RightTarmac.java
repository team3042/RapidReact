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

/** Autonomous Mode (Right Tarmac) - 8 points!! ******************************************************
 * A fancier autonomous routine for scoring 3 cargo when we start in the right tarmac */
public class AutonomousMode_RightTarmac extends SequentialCommandGroup {

  Intake intake = Robot.intake;
  Conveyor conveyor = Robot.conveyor;

  public AutonomousMode_RightTarmac() {
    addCommands(new InstantCommand(intake::extend, intake), // Deploy the intake
                new InstantCommand(conveyor::autoSetPower, conveyor), new Wait(1.5), new InstantCommand(conveyor::stop, conveyor), // Run the conveyor for a specified number of seconds
                new InstantCommand(intake::autoSetPower, intake), // Start running Intake
                new ParallelCommandGroup(Robot.constructTrajectoryCommand("Right_Tarmac", RobotMap.VELOCITY_MAX_MPS, RobotMap.ACCELERATION_MAX_MPS), new Conveyor_Advance()), // Drive our trajectory to intake 2 more cargo
                new InstantCommand(conveyor::autoSetPower, conveyor), new InstantCommand(intake::stop, intake), new Wait(1.5), new InstantCommand(conveyor::stop, conveyor), // Stop the intake and score our additional cargo
                new Drivetrain_GyroStraight(40, 0.5)); // This just makes it easier for the driver to spin and zero the gyro after auto :)
  }
}