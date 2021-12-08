package org.usfirst.frc.team3042.robot.commands.drivetrain;

import org.usfirst.frc.team3042.robot.Robot;

import edu.wpi.first.wpilibj.command.InstantCommand;

/**  Drivetrain Scale Toggle */
public class Drivetrain_Scale_Toggle extends InstantCommand {
  // Scales down the speed of the drivetrain so the driver can make fine-tuned adjustments of robot position
  public Drivetrain_Scale_Toggle() {
    super();
  }

  // Called once when the command executes
  @Override
  protected void initialize() {
    Robot.oi.toggleScale();
  }
}