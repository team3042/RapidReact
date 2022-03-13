package org.usfirst.frc.team3042.robot.commands.autonomous;

import org.usfirst.frc.team3042.robot.Robot;
import org.usfirst.frc.team3042.robot.RobotMap;
import org.usfirst.frc.team3042.robot.commands.Drivetrain_GyroStraight;
import org.usfirst.frc.team3042.robot.commands.autonomous.helperCommands.Wait;
import org.usfirst.frc.team3042.robot.subsystems.Conveyor;
import org.usfirst.frc.team3042.robot.subsystems.Intake;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

/** Autonomous Mode (Left Tarmac) - 6 points! ******************************************************
 * A fancier autonomous routine for scoring 2 cargo when we start in the left tarmac */
public class AutonomousMode_LeftTarmac extends SequentialCommandGroup {

  Intake intake = Robot.intake;
  Conveyor conveyor = Robot.conveyor;

  public AutonomousMode_LeftTarmac() {
    addCommands(new InstantCommand(intake::extend, intake), // Deploy the intake
                new InstantCommand(conveyor::autoSetPower, conveyor), new Wait(2), new InstantCommand(conveyor::stop, conveyor), // Run the conveyor for a specified number of seconds
                new InstantCommand(intake::autoSetPower, intake), // Start running Intake
                Robot.constructTrajectoryCommand("Left_Tarmac", RobotMap.VELOCITY_MAX_MPS, RobotMap.ACCELERATION_MAX_MPS), // Drive our trajectory path to intake more cargo
                new InstantCommand(conveyor::autoSetPower, conveyor), new InstantCommand(intake::stop, intake), new Wait(2), new InstantCommand(conveyor::stop, conveyor), // Stop the intake and score our additional cargo
                new Drivetrain_GyroStraight(40, 0.5)); // This just makes it easier for the driver to spin and zero the gyro after auto :)
  }
}