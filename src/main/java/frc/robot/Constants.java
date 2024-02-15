// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.util.Color;

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

  public static final class kControllers {
    public static final int kPrimaryController = 0;
    public static final int kSecondaryController = 1;
  }

  public static final class kCANID {

  }

  public static final class kDrive {

    public static final double kDriveGearRatio = 6.75;
    public static final double kTurnGearRatio = 150.0 / 7.0;

    public static final double kMaxDriveVelocity = 4.56; // metres per second
    public static final double kMaxTurnAngularVelocity = 10; // rotation per second
    public static final double kMaxTurnAngularAcceleration = 2 * Math.toRadians(360); // rotation per second squared

    public static final double kTranslationDeadband = 0.125;
    public static final double kTargetHeadingDeadband = 0.3;
    public static final double kManualRotationDeadband = 0.2;

    public static final double kDriveRampRate = 0.6;

    public static final double kHeadingSnap = Math.toRadians(45);

    public static final double kHeadingP = 2;
    public static final double kHeadingI = 0;
    public static final double kHeadingD = 0;

  }

  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }

  public static class kDeployment {
    public static final int id_motor = 0;
    public static final int digitalInputPort = 0;
    public static final int voltage = 0;
    public static final int currentLimit = 40;
    public static final int port_irSwitch = 0;

    public static final double kP = 0;
    public static final double kI = 0;
    public static final double kD = 0;

    public static class setpoints {
      public static final double amp_pos = 0;
      public static final double trap_pos = 0;
      public static final double the_end = 0;
    }
  }

  public static class kCartridge {
    public static final int id_motor = 0;
    public static final int voltage = 0;
    public static final int currentLimit = 0;
  }
}
