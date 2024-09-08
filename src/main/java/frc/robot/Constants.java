// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
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

            public static final double ROTATION_P = 5.0;
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

        public static final double KP = 2.4;
        public static final double KI = 0.0;
        public static final double KD = 0.0;
        public static final double KFF = 2.0;

        public static final double MIN_HEIGHT = 0.70;
        public static final double MAX_HEIGHT = 1.40;

        public static final double ELEVATOR_ANGLE = 105;

        public static final double TOLERANCE = 0.01;

        public static final double ELEVATOR_GEARING = 15.0/1.0;
        public static final double ELEVATOR_DRUM_RADIUS = 0.05;
        public static final double ELEVATOR_MASS = Units.lbsToKilograms(11.0);
    }
}
