// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.List;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.kCameras;
import frc.robot.Constants.kPhotonVision;

public class PhotonVision extends SubsystemBase {
  AprilTagFieldLayout aprilTagFieldLayout;
  PhotonCamera frontCamera;
  PhotonCamera backCamera;
  PhotonPoseEstimator poseEstimatorFront;
  PhotonPoseEstimator poseEstimatorBack;

  public PhotonVision() {
    try {
      aprilTagFieldLayout = AprilTagFieldLayout.loadFromResource(kPhotonVision.FIELD_LAYOUT);
    } catch (Exception ignore) {
    }

    // Cameras
    // frontCamera = new PhotonCamera(kCameras.FRONT_CAMERA_ID);
    frontCamera = new PhotonCamera(kCameras.FRONT_CAMERA_ID);
    backCamera = new PhotonCamera(kCameras.BACK_CAMERA_ID);

    // Pose Estimator
    poseEstimatorFront = new PhotonPoseEstimator(aprilTagFieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
        frontCamera,
        kCameras.FRONT_CAMERA_OFFSET);
    poseEstimatorFront.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);

    poseEstimatorBack = new PhotonPoseEstimator(aprilTagFieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
        backCamera,
        kCameras.BACK_CAMERA_OFFSET);
    poseEstimatorBack.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);

    // Smart Dashboard
    initDriverCam();
  }

  /**
   * Returns Optional containing positioning data retrieved through april tags. If
   * there are no april tags visible, an empty optional will be returned.
   * 
   * @param prevEstimatedPose Lsat estimated position //TODO depricate
   * @return Positioning data
   */
  public Optional<EstimatedRobotPose> getEstimatedGlobalPose(Pose2d prevEstimatedPose) {
    poseEstimatorFront.setReferencePose(prevEstimatedPose);
    poseEstimatorBack.setReferencePose(prevEstimatedPose);

    if (frontCamera.isConnected()) {
      Optional<EstimatedRobotPose> photonDataFront = poseEstimatorFront.update();
      Optional<EstimatedRobotPose> photonDataBack = poseEstimatorBack.update();
      Optional<EstimatedRobotPose> photonDataOut;

      if (photonDataFront.isPresent() || photonDataBack.isPresent()) {

        if (photonDataFront.isPresent() && photonDataBack.isPresent()) {
          if (getMeasurementAmbiguity(
              photonDataFront.get().targetsUsed) < (getMeasurementAmbiguity(photonDataBack.get().targetsUsed))) {
            photonDataOut = photonDataFront;
          } else {
            photonDataOut = photonDataBack;
          }
        } else if (photonDataFront.isPresent()) {
          photonDataOut = photonDataFront;
        } else {
          photonDataOut = photonDataBack;
        }

        return isWithinAmbiguityThreshold(photonDataOut.get().targetsUsed, kPhotonVision.AMBIGUITY_THRESHOLD)
            ? photonDataOut
            : Optional.empty();
      }
    } else {
      System.out.println("CAMERA NOT CONNECTED");
    }
    return Optional.empty();
  }

  /**
   * Returns true if given targets are below specified threshold.
   * 
   * @param targets   Targets to check for threshold
   * @param threshold Threshold for targets
   * @return Whether targets are above threshold
   */
  public boolean isWithinAmbiguityThreshold(List<PhotonTrackedTarget> targets, double threshold) {
    for (PhotonTrackedTarget target : targets) {
      if (target.getPoseAmbiguity() >= threshold) {
        // System.out.printf("Target REJECTED | Threshold: %.5f, Value: %.5f\n");
        // threshold, target.getPoseAmbiguity());
        return false;
      }
    }
    return true;
  }

  public double getMeasurementAmbiguity(List<PhotonTrackedTarget> targets) {
    double lowestAmbiguity = targets.get(0).getPoseAmbiguity();
    for (PhotonTrackedTarget target : targets) {
      if (target.getPoseAmbiguity() <= lowestAmbiguity) {
        lowestAmbiguity = target.getPoseAmbiguity();
      }
    }
    return lowestAmbiguity;
  }

  public void initDriverCam() {
    try {
      // CameraServer.startAutomaticCapture(kCameras.kFrontCameraName,
      // kCameras.kFrontCameraURL);
      // CameraServer.startAutomaticCapture(kCameras.kBackCameraName,
      // kCameras.kBackCameraURL);
      // CameraServer.startAutomaticCapture(kCameras.kFrontCameraURL);
      // final HttpCamera camera = new HttpCamera("Camera", kCameras.kFrontCameraURL,
      // HttpCamera.HttpCameraKind.kMJPGStreamer);
      // CameraServer.addCamera(camera);
    } catch (Exception e) {
      System.out.printf("Failed initialize smart dashboard IP cameras: %s\n", e);
    }
  }

  @Override
  public void periodic() {
  }
}
