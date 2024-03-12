// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest.FieldCentricFacingAngle;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.kDrive.kAutoAlign;
import frc.robot.subsystems.Drivetrain;

public class AlignToPose extends Command {
    private final PIDController m_xController;
    private final PIDController m_yController;

    private final Drivetrain sys_drivetrain;
    private final Pose2d targetPose;
    private double notInLineTime;

    /**
     * AlignToPose Constructor
     * 
     * @param targetPose     Target position to navigate to
     * @param sys_Drivetrain Drivetrain
     */
    public AlignToPose(Pose2d targetPose, Drivetrain sys_Drivetrain) {
        this.sys_drivetrain = sys_Drivetrain;
        this.notInLineTime = System.currentTimeMillis();
        this.targetPose = targetPose;

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
        Rotation2d angle = targetPose.getRotation();
        // System.out.println(calculateHeadingDifference(currentPose.getRotation().getRadians(),
        // angle.getRadians()));

        m_xController.setSetpoint(targetPose.getX());
        m_yController.setSetpoint(targetPose.getY());

        // Updating PID controllers
        double xControllerOutput = m_xController.calculate(currentPose.getX());
        double yControllerOutput = m_yController.calculate(currentPose.getY());

        // Applying PID values to drivetrain
        FieldCentricFacingAngle driveCommand = sys_drivetrain.teleopDriveWithAngle
                .withVelocityX(-xControllerOutput)
                .withVelocityY(-yControllerOutput)
                .withTargetDirection(angle);
        sys_drivetrain.runRequest(() -> driveCommand);
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

        double poseDelta = getPoseDelta(currentPose, targetPose);
        double rotationDelta = Math.abs(targetPose.getRotation().getRadians() - currentPose.getRotation().getRadians());
        System.out.println(rotationDelta);

        if (poseDelta >= kAutoAlign.T_CONTROLLER_TOLERANCE || rotationDelta >= kAutoAlign.ROTATION_TOLERANCE) {
            notInLineTime = System.currentTimeMillis();
        } else {
            if ((currentTime - notInLineTime) >= kAutoAlign.REACHED_POSITION_TIMEOUT) {
                return true;
            }
        }
        return false;
    }

    public double calculateHeadingDifference(double heading1, double heading2) {
        double difference = Math.abs(Units.radiansToDegrees(heading1) - Units.radiansToDegrees(heading2));

        if (difference > 180) {
            difference = 360 - difference;
        }

        System.out.printf("TargetHeading: %.1f | CurrentHeading: %.1f | Difference: %.1f\n",
                Units.radiansToDegrees(heading1), Units.radiansToDegrees(heading2),
                difference);
        return difference;
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        moveToPose(targetPose);
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
