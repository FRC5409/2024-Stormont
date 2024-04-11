// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.kCameras;
import frc.robot.Constants.kDrive.kAutoAlign;
import frc.robot.Constants.kPhotonVision;
import java.util.List;
import java.util.Optional;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonTrackedTarget;

public class PhotonVision extends SubsystemBase {
	AprilTagFieldLayout aprilTagFieldLayout;
	PhotonCamera frontCamera, topCamera, backCamera;
	private boolean enableFrontCamera, enableTopCamera, enableBackCamera;
	PhotonPoseEstimator poseEstimatorFront, poseEstimatorTop, poseEstimatorBack;
	private double lastResponseFront, lastResponseTop, lastResponseBack;
	private static PhotonVision instance = null;

	private ShuffleboardTab sb_driveteamtab;

	public PhotonVision() {
		try {
			aprilTagFieldLayout = AprilTagFieldLayout.loadFromResource(kPhotonVision.FIELD_LAYOUT);
		} catch (Exception ignore) {
		}

		// Cameras
		frontCamera = new PhotonCamera(kCameras.FRONT_CAMERA_ID);
		backCamera = new PhotonCamera(kCameras.BACK_CAMERA_ID);
		topCamera = new PhotonCamera(kCameras.TRAP_CAMERA_ID);

		// Front Camera
		poseEstimatorFront = new PhotonPoseEstimator(aprilTagFieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
				frontCamera, kCameras.FRONT_CAMERA_OFFSET);
		poseEstimatorFront.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);

		// Back Camera
		poseEstimatorBack = new PhotonPoseEstimator(aprilTagFieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
				backCamera, kCameras.BACK_CAMERA_OFFSET);
		poseEstimatorBack.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);

		// Top Camera
		poseEstimatorTop = new PhotonPoseEstimator(aprilTagFieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
				topCamera, kCameras.TRAP_CAMERA_OFFSET);
		poseEstimatorTop.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);

		// Shuffleboard
		sb_driveteamtab = Shuffleboard.getTab("Drive team");
		sb_driveteamtab.addBoolean("FrontCamera", () -> frontCamera.isConnected()).withPosition(3, 0);
		sb_driveteamtab.addBoolean("BackCamera", () -> backCamera.isConnected()).withPosition(3, 1);
		sb_driveteamtab.addBoolean("TopCamera", () -> topCamera.isConnected()).withPosition(4, 1);

		this.enableFrontCamera = kPhotonVision.ENABLE_FRONT_CAMERA;
		this.enableBackCamera = kPhotonVision.ENABLE_BACK_CAMERA;
		this.enableTopCamera = kPhotonVision.ENABLE_TOP_CAMERA;
	}

	/**
	 * Returns Optional containing positioning data retrieved through april tags. If
	 * there are no april tags visible, an empty optional will be returned.
	 *
	 * @return Positioning data
	 */
	public Optional<EstimatedRobotPose> getEstimatedGlobalPose(SwerveDrivePoseEstimator currentRobotPose) {
		Optional<EstimatedRobotPose>[] poseEstimates = new Optional[3];
		poseEstimates[0] = getPoseEstimatorUpdate(currentRobotPose, frontCamera, poseEstimatorFront, enableFrontCamera); // Front
		poseEstimates[1] = getPoseEstimatorUpdate(currentRobotPose, backCamera, poseEstimatorBack, enableBackCamera); // Back
		poseEstimates[2] = getPoseEstimatorUpdate(currentRobotPose, topCamera, poseEstimatorTop, enableTopCamera); // Top

		boolean foundMultiTagReading = false;
		double lowestAmbiguity = 1;
		Optional<EstimatedRobotPose> poseEstimateOut = Optional.empty();
		// IN ORDER OF FALLBACK PRIORITY. LAST INDEX IS MOST IMPORTANT

		for (Optional<EstimatedRobotPose> poseEstimate : poseEstimates) {
			// MultiTag-PNP Search
			if (poseEstimate.isPresent()) {
				if (kPhotonVision.DO_MULTITAG_PRIORITIZATION) {
					if (poseEstimate.get().strategy == PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR) {
						double multiTagAmbiguity = getMultiTagAmbiguity(poseEstimate.get().targetsUsed);
						if (multiTagAmbiguity < lowestAmbiguity) {
							poseEstimateOut = poseEstimate;
							lowestAmbiguity = multiTagAmbiguity;
							foundMultiTagReading = true;
							continue;
						}
					}
				}

				// Normal Search
				double singleTagAmbiguity = getMeasurementAmbiguity(poseEstimate.get().targetsUsed);
				if (singleTagAmbiguity < lowestAmbiguity && !foundMultiTagReading) {
					poseEstimateOut = poseEstimate;
					lowestAmbiguity = singleTagAmbiguity;
				}
			}
		}

		// System.out.println("IM UPDATIng");
		return poseEstimateOut;
	}

	/**
	 * Displays status of all cameras on smartdashboard when called
	 */
	public void updateCameraStatus() {
		// Front Camera
		if (!frontCamera.isConnected()) {
			if ((System.currentTimeMillis() - lastResponseFront) > kPhotonVision.CAMERA_STATUS_TIMEOUT) {
				SmartDashboard.putBoolean("[PV] Front", false);
			}
		} else {
			lastResponseFront = System.currentTimeMillis();
			SmartDashboard.putBoolean("[PV] Front", true);
		}

		// Back Camera
		if (!backCamera.isConnected()) {
			if ((System.currentTimeMillis() - lastResponseBack) > kPhotonVision.CAMERA_STATUS_TIMEOUT) {
				SmartDashboard.putBoolean("[PV] Back", false);
			}
		} else {
			lastResponseBack = System.currentTimeMillis();
			SmartDashboard.putBoolean("[PV] Back", true);
		}

		// Top Camera
		if (!topCamera.isConnected()) {
			if ((System.currentTimeMillis() - lastResponseTop) > kPhotonVision.CAMERA_STATUS_TIMEOUT) {
				SmartDashboard.putBoolean("[PV] Top", false);
			} else {
				lastResponseTop = System.currentTimeMillis();
				SmartDashboard.putBoolean("[PV] Top", true);
			}
		}
	}

	/**
	 * Returns a pose estimate from given camera if it is connected and enabled
	 * 
	 * @param camera
	 *            Camera for pose estimate
	 * @param poseEstimator
	 *            Pose estimator to use for update
	 * @param isEnabled
	 *            Is camera enabled
	 * @return Pose estimate
	 */
	private Optional<EstimatedRobotPose> getPoseEstimatorUpdate(SwerveDrivePoseEstimator currentRobotPose,
			PhotonCamera camera, PhotonPoseEstimator poseEstimator, boolean isEnabled) {
		if (camera.isConnected()) {
			Optional<EstimatedRobotPose> photonData = poseEstimator.update();
			if (photonData.isPresent()) {
				if (isWithinAmbiguityThreshold(photonData.get().targetsUsed, kPhotonVision.AMBIGUITY_THRESHOLD)
						&& isEnabled) {

					// Single tag distance filtering
					if (photonData.get().targetsUsed.size() == 1 && kPhotonVision.DO_SINGLE_DISTANCE_FILTERING) {
						if (getPoseDistance(currentRobotPose.getEstimatedPosition(),
								getPoseFromTransform(photonData.get().targetsUsed.get(0)
										.getBestCameraToTarget())) < kPhotonVision.SINGLE_DISTANCE_CUTOFF) {
							return photonData;
						} else {
							System.out.println("CUTOFF TAG");
							System.out.println(getPoseDistance(currentRobotPose.getEstimatedPosition(),
									getPoseFromTransform(photonData.get().targetsUsed.get(0).getBestCameraToTarget())));
							return Optional.empty();
						}
					}

					return photonData;
				}
				// return isWithinAmbiguityThreshold(photonData.get().targetsUsed,
				// kPhotonVision.AMBIGUITY_THRESHOLD) && isEnabled ? photonData :
				// Optional.empty();
			}
		}
		return Optional.empty();
	}

	/**
	 * Returns true if given targets are below specified threshold.
	 *
	 * @param targets
	 *            Targets to check for threshold
	 * @param threshold
	 *            Threshold for targets
	 * @return Whether targets are above threshold
	 */
	public boolean isWithinAmbiguityThreshold(List<PhotonTrackedTarget> targets, double threshold) {
		for (PhotonTrackedTarget target : targets) {
			if (target.getPoseAmbiguity() >= threshold) {
				return false;
			}
		}
		return true;
	}

	/**
	 * Returns lowest ambiguity value from a list of tags
	 * 
	 * @param targets
	 *            Targets to search
	 * @return Lowest ambiguity value found in targets
	 */
	public double getMeasurementAmbiguity(List<PhotonTrackedTarget> targets) {
		double lowestAmbiguity = targets.get(0).getPoseAmbiguity();
		for (PhotonTrackedTarget target : targets) {
			if (target.getPoseAmbiguity() <= lowestAmbiguity) {
				lowestAmbiguity = target.getPoseAmbiguity();
			}
		}
		return lowestAmbiguity;
	}

	/**
	 * Returns average ambiguity value from a list of tags
	 * 
	 * @param targets
	 *            Targets to search
	 * @return Average ambiguity value of targets
	 */
	public double getMultiTagAmbiguity(List<PhotonTrackedTarget> targets) {
		double ambiguity = 0;

		for (PhotonTrackedTarget target : targets) {
			ambiguity += target.getPoseAmbiguity();
		}

		return (ambiguity / targets.size());
	}

	/**
	 * Returns a pose that is offset by a specified amount in field space relative
	 * to the nearest tag.
	 * 
	 * @param sys_drivetrain
	 *            Drivetrain Subsystem
	 * @param offset
	 *            Offset distance
	 * @param targetRotation
	 *            Rotation at which to calculate offset
	 * @return Offset position from nearest tag
	 */
	public Pose2d getNearestTagPoseWithOffset(Drivetrain sys_drivetrain, double offset, double targetRotation) {
		Pose2d currentPose = sys_drivetrain.getAutoRobotPose();
		List<AprilTag> aprilTags = aprilTagFieldLayout.getTags();
		AprilTag closestTag = aprilTagFieldLayout.getTags().get(11);
		double closestDistance = getPoseDistance(currentPose, closestTag.pose.toPose2d());

		// Determining closest tag
		for (AprilTag tag : aprilTags) {
			double distance = getPoseDistance(currentPose, tag.pose.toPose2d());

			if (distance < closestDistance && kAutoAlign.kAprilTags.TRAP_TAG_ROTATIONS.containsKey(tag.ID)) {
				closestDistance = distance;
				closestTag = tag;
			}
		}

		double x = closestTag.pose.getX() + offset * Math.cos(closestTag.pose.getRotation().getAngle());
		double y = closestTag.pose.getY() + offset * Math.sin(closestTag.pose.getRotation().getAngle());
		return new Pose2d(x, y, new Rotation2d(targetRotation));
	}

	/**
	 * Returns the distance between 2 pose2d objects
	 * 
	 * @param pose1
	 *            Position 1
	 * @param pose2
	 *            Position 2
	 * @return Distance between 2 positions
	 */
	private double getPoseDistance(Pose2d pose1, Pose2d pose2) {
		return Math.abs(Math.sqrt(Math.pow(pose2.getX() - pose1.getX(), 2) + Math.pow(pose2.getY() - pose1.getY(), 2)));
	}

	/**
	 * Converts Transform3d to Pose2d
	 * 
	 * @param transform
	 *            Transform3d to convert
	 * @return Pose2d of Transform3d
	 */
	private Pose2d getPoseFromTransform(Transform3d transform) {
		return new Pose2d(transform.getX(), transform.getY(), transform.getRotation().toRotation2d());
	}

	/**
	 * Returns instance of PhotonVision subsystem and creates a new one if not
	 * present
	 * 
	 * @return PhotonVision Subsystem
	 */
	public static PhotonVision getInstance() {
		if (instance == null) {
			instance = new PhotonVision();
		}
		return instance;
	}

	/**
	 * Updates enable status of a specified camera
	 * 
	 * @param isEnabled
	 *            Status to set
	 * @param camera
	 *            Camera to modify
	 */
	public void setCameraEnableStatus(boolean isEnabled, String camera) {
		switch (camera) {
			case "Front" :
				this.enableFrontCamera = isEnabled;
				break;
			case "Back" :
				this.enableBackCamera = isEnabled;
				break;
			case "Top" :
				this.enableTopCamera = isEnabled;
				break;
		}
	}

	@Override
	public void periodic() {
	}
}
