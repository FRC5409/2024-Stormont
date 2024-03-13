// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.Supplier;

import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest.FieldCentric;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.kDrive.kAutoAlign;
import frc.robot.subsystems.Drivetrain;

public class AlignToPose extends Command {
    private final PIDController m_xController;
    private final PIDController m_yController;
    private final PIDController m_rController;

    private final Drivetrain sys_drivetrain;
    private Pose2d targetPose;
    private final Supplier<Pose2d> targetPoseSupplier;
    private double notInLineTime;

    /**
     * AlignToPose Constructor
     * 
     * @param targetPose     Target position to navigate to
     * @param sys_Drivetrain Drivetrain
     */
    public AlignToPose(Supplier<Pose2d> targetPoseSupplier, Drivetrain sys_Drivetrain) {
        this.sys_drivetrain = sys_Drivetrain;
        this.notInLineTime = System.currentTimeMillis();
        this.targetPose = targetPoseSupplier.get();
        this.targetPoseSupplier = targetPoseSupplier;

        // Initializing PID Controllers
        m_xController = new PIDController(kAutoAlign.T_CONTROLLER_P, kAutoAlign.T_CONTROLLER_I,
                kAutoAlign.T_CONTROLLER_D); // TODO
                                            // check
                                            // period
                                            // var
        m_xController.setSetpoint(targetPose.getX());
        m_xController.setTolerance(kAutoAlign.T_CONTROLLER_TOLERANCE);

        m_yController = new PIDController(kAutoAlign.T_CONTROLLER_P, kAutoAlign.T_CONTROLLER_I,
                kAutoAlign.T_CONTROLLER_D);
        m_yController.setSetpoint(targetPose.getY());
        m_yController.setTolerance(kAutoAlign.T_CONTROLLER_TOLERANCE);

        m_rController = new PIDController(kAutoAlign.R_CONTROLLER_P, kAutoAlign.R_CONTROLLER_I,
                kAutoAlign.R_CONTROLLER_D);
        m_rController.setSetpoint(targetPose.getRotation().getRadians());
        // m_rController.setTolerance(kAutoAlign.ROTATION_TOLERANCE);
        m_rController.enableContinuousInput(0, Math.PI * 2);
    }

    /**
     * Returns the combined delta between both axis of 2 points in 2d space
     * 
     * @param pose1 Position 1
     * @param pose2 Position 2
     * @return Sum of XY difference between 2 points
     */
    public double getPoseDelta(Pose2d pose1, Pose2d pose2) {
        return Math.abs(pose1.getX() - pose2.getX()) + Math.abs(pose1.getY() - pose2.getY());
    }

    /**
     * Moves robot to target position on the field
     * 
     * @param targetPose Target position to move to
     */
    public void moveToPose(Pose2d targetPose) {
        Pose2d currentPose = sys_drivetrain.getAutoRobotPose();
        // Rotation2d angle = targetPose.getRotation();

        m_xController.setSetpoint(targetPose.getX());
        m_yController.setSetpoint(targetPose.getY());

        // Updating PID controllers
        double xControllerOutput = applyTolerance(applyFeatForward(m_xController.calculate(currentPose.getX()),
                kAutoAlign.T_CONTROLLER_FF), currentPose.getX(), targetPose.getX(), kAutoAlign.T_CONTROLLER_TOLERANCE);
        double yControllerOutput = applyTolerance(applyFeatForward(m_yController.calculate(currentPose.getY()),
                kAutoAlign.T_CONTROLLER_FF), currentPose.getY(), targetPose.getY(), kAutoAlign.T_CONTROLLER_TOLERANCE);
        double rControllerOutput = applyTolerance(
                applyFeatForward(m_rController.calculate(currentPose.getRotation().getRadians()),
                        kAutoAlign.R_CONTROLLER_FF),
                currentPose.getRotation().getRadians(),
                targetPose.getRotation().getRadians(), kAutoAlign.ROTATION_TOLERANCE);

        // Invert for field centric if alliance is red
        if (DriverStation.getAlliance().isPresent()) {
            if (DriverStation.getAlliance().get() == Alliance.Red) {
                xControllerOutput = -xControllerOutput;
                yControllerOutput = -yControllerOutput;
            }
        }

        // Applying PID values to drivetrain
        FieldCentric driveCommand = sys_drivetrain.teleopDrive
                .withVelocityX(xControllerOutput)
                .withVelocityY(yControllerOutput)
                .withRotationalRate(rControllerOutput);

        sys_drivetrain.runRequest(() -> driveCommand);
    }

    private double applyFeatForward(double input, double featForward) {
        if (input < 0) {
            return input - featForward;
        } else if (input > 0) {
            return input + featForward;
        }
        return 0;
    }

    /**
     * Returns true if the robot has been aligned with the target position for more
     * than 500ms
     * 
     * @param targetPose Target position
     * @return If aligned with target position
     */
    private boolean isAligned(Pose2d targetPose) {
        double currentTime = System.currentTimeMillis();
        Pose2d currentPose = sys_drivetrain.getAutoRobotPose();

        // double poseDelta = getPoseDelta(currentPose, targetPose);
        double poseDelta = getPoseDistance(currentPose, targetPose);
        double rotationDelta = Math.abs(targetPose.getRotation().getRadians() - currentPose.getRotation().getRadians());

        if (poseDelta >= kAutoAlign.REACHED_POSITION_TOLERANCE || rotationDelta >= kAutoAlign.ROTATION_TOLERANCE) {
            notInLineTime = System.currentTimeMillis();
        } else {
            if ((currentTime - notInLineTime) >= kAutoAlign.REACHED_POSITION_TIMEOUT) {
                return true;
            }
        }
        return false;
    }

    private double getPoseDistance(Pose2d pose1, Pose2d pose2) {
        return Math.sqrt(Math.pow(pose2.getX() - pose1.getX(), 2) + Math.pow(pose2.getY() - pose1.getY(), 2));
    }

    public double calculateHeadingDifference(double heading1, double heading2) {
        double difference = Math.abs(Units.radiansToDegrees(heading1) - Units.radiansToDegrees(heading2));

        if (difference > 180) {
            difference = 360 - difference;
        }
        return difference;
    }

    public double applyTolerance(double input, double current, double target, double tolerance) {
        if (Math.abs(current - target) < tolerance) {
            return 0;
        }
        return input;
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        this.targetPose = targetPoseSupplier.get();
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        moveToPose(targetPose);
        // PhotonVision.getInstance().getNearestTagPoseWithOffset(sys_drivetrain, 0);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return isAligned(targetPose);
    }
}
