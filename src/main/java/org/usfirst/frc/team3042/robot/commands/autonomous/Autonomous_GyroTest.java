package org.usfirst.frc.team3042.robot.commands.autonomous;

import org.usfirst.frc.team3042.robot.commands.Drivetrain_GyroStraight;
import org.usfirst.frc.team3042.robot.commands.Drivetrain_GyroTurn;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class Autonomous_GyroTest extends SequentialCommandGroup {

  /** Made for testing the GyroStraight() and GyroTurn() commands */
  public Autonomous_GyroTest() {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(new Drivetrain_GyroStraight(20, 50), new Drivetrain_GyroTurn(90));
  }
}