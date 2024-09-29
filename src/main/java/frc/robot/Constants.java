// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.util.GeometryUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

    public static class kController {
        public static final int kDriverControllerPort = 0;

        public static final double kJoystickDeadband = 0.05;
        public static final double kTriggerDeadband = 0.05;
    }

    public static class kDrive {
        public static final int CURRENT_LIMIT = 100;

        public static final double MAX_CHASSIS_SPEED = 4.56;
        public static final double MAX_CHASSIS_ACCELERATION = 3.0;
        public static final double MAX_ROTATION_SPEED = 10.0;
        public static final double MAX_ROTATION_ACCELERATION = Math.toRadians(720.0);

        public static class kPID {
            public static final double TRANSLATION_P = 5.0;
            public static final double TRANSLATION_I = 0.0;
            public static final double TRANSLATION_D = 0.0;

            public static final double ROTATION_P = 7.0;
            public static final double ROTATION_I = 0.0;
            public static final double ROTATION_D = 0.0;
        }
    }

    public static class kAuto {
        public static final Pose2d FAR_SHOOTING_POSE = new Pose2d(new Translation2d(4.03, 5.44), Rotation2d.fromDegrees(0.0));
        public static final Pose2d CLOSE_SHOOTING_POSE = new Pose2d(new Translation2d(1.92, 5.54), Rotation2d.fromDegrees(0.0));
        public static final Pose2d[] NOTES = {
            // Close notes
            new Pose2d(new Translation2d(2.89, 7.00), Rotation2d.fromDegrees(0.0)),
            new Pose2d(new Translation2d(2.89, 5.54), Rotation2d.fromDegrees(0.0)),
            new Pose2d(new Translation2d(2.89, 4.10), Rotation2d.fromDegrees(0.0)),

            // Mid notes
            new Pose2d(new Translation2d(8.29, 7.44), Rotation2d.fromDegrees(0.0)),
            new Pose2d(new Translation2d(8.29, 5.78), Rotation2d.fromDegrees(0.0)),
            new Pose2d(new Translation2d(8.29, 4.12), Rotation2d.fromDegrees(0.0)),
            new Pose2d(new Translation2d(8.29, 2.46), Rotation2d.fromDegrees(0.0)),
            new Pose2d(new Translation2d(8.29, 0.80), Rotation2d.fromDegrees(0.0))
        };

        public static final Translation3d BLUE_SPEAKER = new Translation3d(0.025, 5.55, 2.1);
        public static final Translation3d RED_SPEAKER = new Translation3d(16.117, 5.55, 2.1);

        public static final Pose2d[] BLUE_STAGES = {
            new Pose2d(4.43, 4.92, Rotation2d.fromDegrees(120)), // LEFT
            new Pose2d(4.43, 3.26, Rotation2d.fromDegrees(-120)), // RIGHT
            new Pose2d(5.80, 4.10, Rotation2d.fromDegrees(0)) // CENTER
        };

        public static final Pose2d[] RED_STAGES = {
            GeometryUtil.flipFieldPose(BLUE_STAGES[0]).plus(new Transform2d(0, 0, Rotation2d.fromDegrees(180))), // LEFT
            GeometryUtil.flipFieldPose(BLUE_STAGES[1]).plus(new Transform2d(0, 0, Rotation2d.fromDegrees(180))), // RIGHT
            GeometryUtil.flipFieldPose(BLUE_STAGES[2]).plus(new Transform2d(0, 0, Rotation2d.fromDegrees(180)))  // CENTER
        };

        public static final Pose2d BLUE_AMP = new Pose2d(1.80, 7.66, Rotation2d.fromDegrees(-90));
        public static final Pose2d RED_AMP = new Pose2d(14.69, 7.66, Rotation2d.fromDegrees(90));
    }

    public static final class kIntake {
        public static final int INTAKE_MOTOR_ID = 20;
        
        public static final int IR_CHANNEL = 2;

		public static final int CURRENT_LIMIT = 40;

		public static final double VOLTAGE = 7;
	}

    public static final class kIndexer {
        public static final int CANID = 22;

		public static final int CURRENT_LIMIT = 30;

		public static final double VOLTAGE = 8;
	}

    public static final class kCartridge {
        public static final int CANID = 16;

        public static final int IR_CHANNEL = 1;
        
		public static final int CURRENT_LIMIT = 30;

		public static final double VOLTAGE = 7;
	}

    public static final class kDeployment {
        public static final int DEPLOYMENT_ID = 15;
        public static final int CURRENT_LIMIT = 40;

        public static final double KP = 6.0;
        public static final double KI = 0.0;
        public static final double KD = 0.0;
        public static final double KFF = 2.4;

        public static final double MIN_HEIGHT = Units.inchesToMeters(17.55);
        public static final double MAX_HEIGHT = Units.inchesToMeters(17.55 * 2);

        public static final double ELEVATOR_ANGLE = 105;

        public static final double TOLERANCE = 0.01;

        public static final double ELEVATOR_GEARING = 10.0/1.0;
        public static final double ELEVATOR_DRUM_RADIUS = Units.inchesToMeters(0.944);
        public static final double ELEVATOR_MASS = Units.lbsToKilograms(11.0);
    }

    public static final class kVision {
        public static final String name = "limelight";
        public static final Pose3d limelightPoseOffset = new Pose3d(
            0,
            0,
            0,
            new Rotation3d(
                0,
                0,
                0
            )
        );
    }
}
