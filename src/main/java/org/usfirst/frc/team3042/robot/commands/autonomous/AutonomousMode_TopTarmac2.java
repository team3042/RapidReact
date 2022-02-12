package org.usfirst.frc.team3042.robot.commands.autonomous;

import org.usfirst.frc.team3042.robot.commands.Conveyor_Advance;
import org.usfirst.frc.team3042.robot.commands.Conveyor_Run;
import org.usfirst.frc.team3042.robot.commands.Drivetrain_Trajectory;
import org.usfirst.frc.team3042.robot.commands.Intake_Intake;
import org.usfirst.frc.team3042.robot.commands.Intake_Toggle;
import org.usfirst.frc.team3042.robot.commands.autonomous.helperCommands.Wait;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

/** Autonomous Mode Top Tarmac 2 ******************************************************
 * This is an autonomous routine for tournaments! 
 * What it does is score our starting cargo, then deploy the intake, start running the intake, and drive our trajectory, then stop the intake and score the additional two cargo */
public class AutonomousMode_TopTarmac2 extends SequentialCommandGroup {

  public AutonomousMode_TopTarmac2() {
    addCommands(new Conveyor_Run(1), new Wait(3), new Conveyor_Run(0),
                new Intake_Toggle(),
                new ParallelCommandGroup(new Intake_Intake(1), new Conveyor_Advance()),
                new Drivetrain_Trajectory("Top2_Path2"),
                new ParallelCommandGroup(new Conveyor_Run(1), new Intake_Intake(0)));
  }
}