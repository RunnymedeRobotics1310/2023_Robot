// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

    public static final class OiConstants {
        public static final int DRIVER_CONTROLLER_PORT = 0;
    }

    public static final class AutoConstants {
        public static final String AUTO_PATTERN_DO_NOTHING = "Do nothing";
        public static final String AUTO_PATTERN_SHOOT = "Shoot";
        public static final String AUTO_PATTERN_MID_DIRECT_CHARGE = "Start in the middle, head directly to the charge station";
    }

    public static final class IntakeConstants {

        public static final int     INTAKE_MOTOR_ADDRESS  = 30;
        public static final boolean INTAKE_MOTOR_REVERSED = false;

        public static final int INTAKE_ROLLER_PISTON_ADDRESS = 0;
        public static final int INTAKE_HOOD_PISTON_ADDRESS   = 2;
    }

    public static final class ShooterConstants {

        public static final int     SHOOTER_MOTOR_ADDRESS  = 51;
        public static final boolean SHOOTER_MOTOR_REVERSED = true;

        public static final int     KICKER_MOTOR_ADDRESS  = 50;
        public static final boolean KICKER_MOTOR_REVERSED = true;

        public static final int BALL_SENSOR_ADDRESS = 0;

        public static final double SHOOT_HIGH_SPEED = .75d;
        public static final double SHOOT_LOW_SPEED  = .30d;
    }
}
