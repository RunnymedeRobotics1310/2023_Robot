// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot;

import frc.robot.Constants.GameConstants.GamePiece;
import frc.robot.Constants.GameConstants.ScoringRow;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose.
 * <p>
 * All constants should be declared globally (i.e. public static).
 * <br>
 * Do not put anything functional in this class.
 * <p>
 * It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

    // Global constants
    public static final double DEFAULT_COMMAND_TIMEOUT_SECONDS = 5;

    public static final class GameConstants {

        public static enum GamePiece {

            // TODO: set these values
            CUBE(65), CONE(124), NONE(0);

            public final double pincherEncoderCount;

            GamePiece(double pincherEncoderCount) {
                this.pincherEncoderCount = pincherEncoderCount;
            }
        }

        public static enum ScoringRow {
            BOTTOM, MIDDLE, TOP
        }

        public static enum PickupLocation {
            FLOOR, DOUBLE_SUBSTATION
        }

        public static enum Zone {
            COMMUNITY, FIELD, LOADING
        }
    }

    public static final class AutoConstants {

        public static enum AutoLane {
            BOTTOM, MIDDLE, TOP
        }

        public static enum Orientation {
            FACE_GRID, FACE_FIELD
        }

        public static enum AutoAction {
            DO_NOTHING,
            SCORE_BOTTOM, SCORE_MIDDLE, SCORE_TOP, // Game Piece Scoring Locations
            EXIT_ZONE, PICK_UP_CUBE, PICK_UP_CONE, // Actions when exiting zone
            BALANCE
        }
    }

    public static final class DriveConstants {

        public static enum DriveMode {
            TANK, ARCADE, QUENTIN, DUAL_STICK_ARCADE;
        }

        public static final int     LEFT_MOTOR_PORT      = 10;
        public static final int     RIGHT_MOTOR_PORT     = 20;

        public static final boolean LEFT_MOTOR_REVERSED  = false;
        public static final boolean RIGHT_MOTOR_REVERSED = true;

        public static final double  CM_PER_ENCODER_COUNT = 3.503;

        public static final boolean GYRO_REVERSED        = false;
    }

    public static final class OiConstants {

        public static final int DRIVER_CONTROLLER_PORT   = 0;
        public static final int OPERATOR_CONTROLLER_PORT = 1;
    }

    public static final class VisionConstants {

        /** Time to switch pipelines and acquire a new vision target */
        public static final double VISION_SWITCH_TIME_SEC = .25;

        /** The camera view */
        public enum CameraView {
            /** Point at an angle to view the intake */
            LOW,
            /** Point towards the high goal */
            HIGH,
            /** Neither high nor low */
            IN_BETWEEN
        }

        public static final int    CAMERA_ANGLE_MOTOR_PORT         = 50;

        // Set the current limit of the Neo 550 to 20A
        public static final int    CAMERA_MOTOR_CURRENT_LIMIT      = 20;

        /** The maximum speed to drive the camera - manually or automatically */
        public static final double MAX_CAMERA_MOTOR_SPEED          = 1;
        public static final double MAX_CAMERA_SLOW_ZONE_SPEED      = .2;

        public static final double CAMERA_DOWN_LIMIT_ENCODER_VALUE = -255;
        public static final double CAMERA_POSITION_TOLERANCE       = 3;
        public static final double CAMERA_POSITION_SLOW_ZONE       = 15;
    }

    public static class ArmPosition {

        public final double angle, extension;

        ArmPosition(double angle, double extension) {
            this.angle     = angle;
            this.extension = extension;
        }
    }

    public static final class ArmConstants {

        /*
         * Arm Lift Constants
         */
        public static final int         ARM_LIFT_MOTOR_PORT                = 30;
        public static final boolean     ARM_LIFT_MOTOR_REVERSED            = false;

        public static final int         ARM_DOWN_LIMIT_SWITCH_DIO_PORT     = 0;

        /** The maximum speed to drive the arm lift - manually or automatically */
        public static final double      MAX_LIFT_SPEED                     = .25;
        public static final double      MAX_LIFT_SLOW_ZONE_SPEED           = .1;

        public static final double      ARM_LIFT_LIMIT_ENCODER_VALUE       = 16;
        public static final double      ARM_LIFT_ANGLE_TOLERANCE_DEGREES   = 2;
        public static final double      ARM_LIFT_SLOW_ZONE_DEGREES         = 5;

        public static final double      CLEAR_FRAME_ARM_ANGLE              = 30;                                  // TODO: get
                                                                                                                  // real value

        /*
         * Arm Angle Constants
         */

        /** Hard stop angle where 0 = straight down, and 90 = parallel to floor */
        public static final double      ARM_DOWN_ANGLE_DEGREES             = 23;

        /** Down position is 23 degrees, and there are 12.5 encoder counts to horizontal, 90 deg */
        public static final double      ARM_DEGREES_PER_ENCODER_COUNT      = (90 - ARM_DOWN_ANGLE_DEGREES) / 12.5;

        // Calculate the lift limit in degrees instead of encoder counts */
        public static final double      ARM_LIFT_LIMIT_DEGREES             = (ARM_LIFT_LIMIT_ENCODER_VALUE
            * ARM_DEGREES_PER_ENCODER_COUNT) + ARM_DOWN_ANGLE_DEGREES;


        /*
         * Arm Extender Constants
         */
        public static final int         ARM_EXTEND_MOTOR_PORT              = 35;
        public static final boolean     ARM_EXTEND_MOTOR_REVERSED          = true;

        public static final double      MAX_EXTEND_SPEED                   = 1;
        public static final double      MAX_EXTEND_SLOW_ZONE_SPEED         = .15;

        public static final double      ARM_EXTEND_LIMIT_ENCODER_VALUE     = 56.0;
        public static final double      ARM_EXTEND_POSITION_TOLERANCE      = 2;
        public static final double      ARM_EXTEND_SLOW_ZONE_ENCODER_VALUE = 7;


        /*
         * Pincher Constants
         */
        public static final int         PINCHER_MOTOR_PORT                 = 40;
        public static final boolean     PINCHER_MOTOR_REVERSED             = true;

        // Set the current limit of the Neo 550 to 20A
        public static final int         PINCHER_MOTOR_CURRENT_LIMIT        = 20;

        public static final double      MAX_PINCHER_SPEED                  = 1;
        public static final double      MAX_PINCHER_SLOW_ZONE_SPEED        = .15;
        public static final double      PINCHER_CLOSE_LIMIT_ENCODER_VALUE  = 126;
        public static final double      PINCHER_POSITION_TOLERANCE         = 2;
        public static final double      PINCHER_SLOW_ZONE_ENCODER_VALUE    = 10;


        /*
         * Scoring Constants
         */
        public static final ArmPosition SCORE_TOP_CONE_POSITION            = new ArmPosition(100, 50);
        public static final ArmPosition SCORE_TOP_CUBE_POSITION            = new ArmPosition(100, 50);

        public static final ArmPosition SCORE_MIDDLE_CONE_POSITION         = new ArmPosition(75, 25);
        public static final ArmPosition SCORE_MIDDLE_CUBE_POSITION         = new ArmPosition(75, 25);

        public static final ArmPosition SCORE_BOTTOM_CONE_POSITION         = new ArmPosition(40, 10);
        public static final ArmPosition SCORE_BOTTOM_CUBE_POSITION         = new ArmPosition(40, 10);

        // Pickup constants
        public static final ArmPosition GROUND_PICKUP_POSITION             = new ArmPosition(30, 10);
        public static final ArmPosition SUBSTATION_PICKUP_POSITION         = new ArmPosition(30, 10);

        // Helper routine to get a scoring position
        public static ArmPosition getScoringPosition(GamePiece gamePiece, ScoringRow scoringRow) {

            if (gamePiece == GamePiece.CONE) {

                switch (scoringRow) {
                case TOP:
                    return SCORE_TOP_CONE_POSITION;
                case MIDDLE:
                    return SCORE_MIDDLE_CONE_POSITION;
                case BOTTOM:
                    return SCORE_BOTTOM_CONE_POSITION;
                }

            }
            else if (gamePiece == GamePiece.CUBE) {

                switch (scoringRow) {
                case TOP:
                    return SCORE_TOP_CUBE_POSITION;
                case MIDDLE:
                    return SCORE_MIDDLE_CUBE_POSITION;
                case BOTTOM:
                    return SCORE_BOTTOM_CUBE_POSITION;
                }

            }

            // If we got here there is an error
            System.out.println("No known scoring position for game piece " + gamePiece
                + " scoring row " + scoringRow);

            // Return the default scoring position of bottom cube
            return SCORE_BOTTOM_CUBE_POSITION;
        }
    }
}
