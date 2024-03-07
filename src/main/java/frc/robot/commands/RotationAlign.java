// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.List;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.kDrive.kAutoAlign;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.PhotonVision;

public class RotationAlign extends Command {
  private final PIDController rotationController;
  private final PhotonVision sys_PhotonVision;
  private final Drivetrain sys_Drivetrain;

  public RotationAlign(PhotonVision sys_PhotonVision, Drivetrain sys_Drivetrain) {
    this.sys_PhotonVision = sys_PhotonVision;
    this.sys_Drivetrain = sys_Drivetrain;

    rotationController = new PIDController(kAutoAlign.kRotation.R_Controller_P, kAutoAlign.kRotation.R_Controller_I,
        kAutoAlign.kRotation.R_Controller_D);
    rotationController.setTolerance(kAutoAlign.kRotation.R_Controller_TOLARANCE);
    rotationController.enableContinuousInput(0, (Math.PI * 2));
    rotationController.setSetpoint(getNearestTagRotation());
  }

  public double getNearestTagRotation() {
    AprilTagFieldLayout fieldLayout = sys_PhotonVision.getFieldLayout();
    List<AprilTag> aprilTags = fieldLayout.getTags();
    Pose2d currentPose = sys_Drivetrain.getRobotPose();
    double shortestDistance = getPoseDelta(currentPose, aprilTags.get(0).pose.toPose2d());
    AprilTag nearestTag = aprilTags.get(0);

    for (AprilTag aprilTag : aprilTags) {
      Pose2d aprilTagPose = aprilTag.pose.toPose2d();
      double distance = getPoseDelta(currentPose, aprilTagPose);

      if (distance < shortestDistance) {
        shortestDistance = distance;
        nearestTag = aprilTag;

        System.out.printf("Aligning to tag %s\n", aprilTag.ID);
      }
    }

    return nearestTag.pose.getRotation().getZ();
  }

  public void applyRotationSpeed() {
    Pose2d currentPose = sys_Drivetrain.getRobotPose();
    double calculatedRotationRate = rotationController.calculate(currentPose.getRotation().getRadians(),
        getNearestTagRotation());

    // APPLY ROTATION SPEED TO DRIVETRAIN
  }

  public double getPoseDelta(Pose2d pose1, Pose2d pose2) {
    return Math.abs(pose1.getX() - pose2.getX()) + Math.abs(pose1.getY() - pose2.getY());
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
