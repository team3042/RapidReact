package org.usfirst.frc.team3042.robot.commands.autonomous.helperCommands;

import java.util.function.Consumer;
import java.util.function.Supplier;

import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPlannerTrajectory.PathPlannerState;

import org.usfirst.frc.team3042.robot.Robot;

import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.MecanumDriveKinematics;
import edu.wpi.first.math.kinematics.MecanumDriveWheelSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Subsystem;

@SuppressWarnings("MemberName")
public class PPMecanumControllerCommand extends CommandBase {
  private final Timer m_timer = new Timer();
  private final PathPlannerTrajectory m_trajectory;
  private final Supplier<Pose2d> m_pose;
  private final MecanumDriveKinematics m_kinematics;
  private final HolonomicDriveController m_controller;
  private final Consumer< MecanumDriveWheelSpeeds> m_outputWheelSpeeds;

  @SuppressWarnings("ParameterName")
  public PPMecanumControllerCommand(
    PathPlannerTrajectory trajectory,
    Supplier<Pose2d> pose,
    MecanumDriveKinematics kinematics,
    PIDController xController,
    PIDController yController,
    ProfiledPIDController thetaController,
    Consumer< MecanumDriveWheelSpeeds> outputWheelSpeeds,
    Subsystem... requirements) {

    m_trajectory = trajectory;
    m_pose = pose;
    m_kinematics = kinematics;

    m_controller = new HolonomicDriveController(xController, yController, thetaController);

    m_outputWheelSpeeds = outputWheelSpeeds;

    addRequirements(requirements);
  }

  @Override
  public void initialize() {
    PathPlannerState initialState = (PathPlannerState)m_trajectory.sample(0); // Define the initial state of the trajectory

    Robot.drivetrain.resetOdometry(new Pose2d(initialState.poseMeters.getTranslation(), initialState.holonomicRotation));

    m_timer.reset();
    m_timer.start();
  }

  @Override
  @SuppressWarnings("LocalVariableName")
  public void execute() {
    double curTime = m_timer.get();
    var desiredState = (PathPlannerState) m_trajectory.sample(curTime);

    var targetChassisSpeeds = m_controller.calculate(m_pose.get(), desiredState, desiredState.holonomicRotation);
    var targetWheelSpeeds = m_kinematics.toWheelSpeeds(targetChassisSpeeds);

    m_outputWheelSpeeds.accept(targetWheelSpeeds);
  }

  @Override
  public void end(boolean interrupted) {
    m_timer.stop();
  }

  @Override
  public boolean isFinished() {
    return m_timer.hasElapsed(m_trajectory.getTotalTimeSeconds());
  }
}