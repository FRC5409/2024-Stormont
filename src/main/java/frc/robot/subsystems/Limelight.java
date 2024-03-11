// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;
import frc.robot.LimelightHelpers.PoseEstimate;
import frc.robot.Constants.kLimelight;

public class Limelight extends SubsystemBase {
  /** Creates a new Limelight. */
  public Limelight() {
  }

  public PoseEstimate getEstimatedPose() {
    return LimelightHelpers.getBotPoseEstimate_wpiBlue(kLimelight.LIMELIGHT_ID);

    // return
    // LimelightHelpers.getBotPoseEstimate_wpiBlue(kLimelight.LIMELIGHT_ID).pose;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
