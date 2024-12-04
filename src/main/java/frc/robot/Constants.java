// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;

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

    public static enum kMode {
        /**
         * A REAL robot that exists/should be created with logging
         */
        REAL,
        /**
         * A REAL robot that exists/should be created with NO logging
         */
        REAL_NO_LOG,
        /**
         * A Replay of a previous robot values
         */
        REPLAY,
        /**
         * A SIMULATED robot that exists/should be created
         */
        SIM
    }

    public static final kMode REAL_MODE = kMode.REAL;
    public static final kMode SIM_MODE = kMode.SIM;

    /**
     * Gets the mode of the robot
     * WARNING: If the robot is booting and REAL_MODE was not set correctly for a comp match
     * it will return the set REAL_MODE until it connects to a FMS!
     * @return the given robot mode
     */
    public static kMode getMode() {
        if (RobotBase.isReal())
            return DriverStation.isFMSAttached() ? kMode.REAL : REAL_MODE;
        else 
            return SIM_MODE;
    }

    public static class kController {
        public static final int kDriverControllerPort = 0;
        public static final int kSecondaryController = 1;

        public static final double kJoystickDeadband = 0.05;
        public static final double kTriggerDeadband = 0.05;
    }

    public static class kClimber {
        public static class kSpark {
            public static final int ID = 0;

            public static final boolean INVERTED = false;
            public static final int CURRENT_LIMIT = 30;
            public static final double CONVERSION_FACTOR = 1;

            public static final double kP = 0.0;
            public static final double kI = 0.0;
            public static final double kD = 0.0;
            public static final double kFF = 0.0;
        }

        public static class kTalon {
            public static final int ID = 0;

            public static final boolean INVERTED = false;
            public static final int CURRENT_LIMIT = 30;
            public static final double CONVERSION_FACTOR = 1;

            public static final double kP = 0.0;
            public static final double kI = 0.0;
            public static final double kD = 0.0;
        }
    }
}
