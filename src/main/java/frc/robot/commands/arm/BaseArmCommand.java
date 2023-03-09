package frc.robot.commands.arm;

import static frc.robot.Constants.ArmConstants.*;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Constants.ArmConstants;
import frc.robot.subsystems.ArmSubsystem;

abstract class BaseArmCommand extends CommandBase {

    protected final ArmSubsystem armSubsystem;

    protected BaseArmCommand(ArmSubsystem armSubsystem) {
        this.armSubsystem = armSubsystem;
        addRequirements(armSubsystem);
    }

    protected final void printArmState() {
        System.out.println("BaseArmCommand: armSubsystem.getHeldGamePiece: " + armSubsystem.getHeldGamePiece());
        System.out.println("BaseArmCommand: Arm angle: " + armSubsystem.getArmLiftAngle());
        System.out.println("BaseArmCommand: Arm extent: " + armSubsystem.getArmExtendEncoder());
        System.out.println("BaseArmCommand: Pincher: " + armSubsystem.getPincherEncoder());
    }

    protected final void stopArmMotors() {
        armSubsystem.stop();
    }

    /**
     * Convenience method to move the arm to the bottom position (lowest arm angle).
     *
     * @return {@code true} if at the bottom, {@code false} otherwise
     */
    protected boolean lowerArmToBottom() {
        return moveArmLiftToAngle(ArmConstants.ARM_DOWN_ANGLE_DEGREES);
    }

    /**
     * Move the motor to a specified encoder count
     *
     * @param targetAngle the angle in degrees
     * @return true if at the desired location, false if still moving to that point
     */
    protected final boolean moveArmLiftToAngle(double targetAngle) {
        armSubsystem.moveArmLiftToAngle(targetAngle);
        return armSubsystem.isArmAtLiftAngle(targetAngle);
    }

    /**
     * Convenience method to fully retract the arm.
     *
     * @return {@code true} if in frame, {@code false} otherwise
     */
    protected boolean retractArm() {
        return moveArmExtendToEncoderCount(0, MAX_EXTEND_SPEED);
    }

    /**
     * Move the motor to a specified encoder count
     *
     * @param targetCount the count to get to
     * @param speed the absolute value of the speed at which you should move
     * @return true if at the desired location, false if still moving to that point
     */
    protected final boolean moveArmExtendToEncoderCount(double targetCount, double speed) {

        SmartDashboard.putBoolean("At Extension Position", armSubsystem.isAtExtendPosition(targetCount));

        if (targetCount <= 0) {
            targetCount = -5; // compensate for the encoder at zero not aligning with the limit switch
            if (armSubsystem.isArmRetracted()) {
                armSubsystem.setArmExtendSpeed(0);
                return true;
            }
        }
        else {
            if (armSubsystem.isAtExtendPosition(targetCount)) {
                armSubsystem.setArmExtendSpeed(0);
                return true;
            }
        }

        double absSpd = Math.abs(speed);
        double gap    = targetCount - armSubsystem.getArmExtendEncoder();

        // Special logic for the slow zone when the target encoder count <=0
        if (targetCount <= 0) {

            if (armSubsystem.getArmExtendEncoder() < ArmConstants.ARM_EXTEND_SLOW_ZONE_ENCODER_VALUE) {

                absSpd = Math.min(absSpd, ArmConstants.MAX_EXTEND_SLOW_ZONE_SPEED);
            }
        }
        else {
            // Determine whether to slow down because we are close to the target.
            if (Math.abs(gap) < ArmConstants.ARM_EXTEND_SLOW_ZONE_ENCODER_VALUE) {

                absSpd = Math.min(absSpd, ArmConstants.MAX_EXTEND_SLOW_ZONE_SPEED);
            }
        }

        armSubsystem.setArmExtendSpeed(gap < 0 ? -absSpd : absSpd);

        return false;

    }

    /**
     * Convenience method to set the pincher inside the frame for storage.
     *
     * @return {@code true} if in frame, {@code false} otherwise
     */
    protected boolean movePincherInsideFrame() {
        return movePincherToEncoderCount(ArmConstants.MIN_PINCHER_INSIDE_FRAME_POSITION);
    }

    /**
     * Move the pincher motor to a specified encoder count
     *
     * @param targetCount the count to get to
     * @return true if at the desired location, false if still moving to that point
     */
    protected final boolean movePincherToEncoderCount(double targetCount) {

        SmartDashboard.putBoolean("At Pincher Position", armSubsystem.isAtPincherPosition(targetCount));

        if (targetCount == 0) {
            if (armSubsystem.isPincherOpen()) {
                armSubsystem.setPincherSpeed(0);
                return true;
            }
        }
        else {
            if (armSubsystem.isAtPincherPosition(targetCount)) {
                armSubsystem.setPincherSpeed(0);
                return true;
            }
        }

        double gap   = targetCount - armSubsystem.getPincherEncoder();
        double speed = MAX_PINCHER_SPEED;

        // Determine whether to slow down because we are close to the target.
        if (Math.abs(gap) < ArmConstants.PINCHER_SLOW_ZONE_ENCODER_VALUE) {

            // If opening (-ve gap), then slow down when close to the target
            if (gap < 0) {
                speed = ArmConstants.MAX_PINCHER_SLOW_ZONE_SPEED;
            }
            else {
                // If closing (+ve gap) and no game piece is detected, then slow down when
                // close to the target
                if (!armSubsystem.isGamePieceDetected()) {
                    speed = ArmConstants.MAX_PINCHER_SLOW_ZONE_SPEED;
                }
            }
        }

        armSubsystem.setPincherSpeed(gap < 0 ? -speed : speed);

        return false;
    }

    /**
     * Move the motor to a specified encoder count
     *
     * @return true if at the desired location, false if still moving to that point
     */
    protected final boolean openPincher() {

        armSubsystem.setPincherSpeed(-MAX_PINCHER_SPEED);

        if (armSubsystem.isPincherOpen()) {
            armSubsystem.setPincherSpeed(0);
            return true;
        }

        return false;
    }

    private enum CompactState {
        PREPARING, RETRACTING, LOWERING, COMPACT_POSE;
    }

    private CompactState compactState = null;

    protected final boolean isCompactPose() {
        return armSubsystem.isArmDown()
            && armSubsystem.isArmRetracted() && armSubsystem.isPincherInsideFrame();
    }

    /**
     * Safely move the arm from whatever position it is in to the compact pose, which has the arm down, retracted, and with
     * pinchers closed.
     *
     * @return true if in the compact pose, false if still moving there
     */
    protected final boolean moveToCompactPose() {

        // Figure out initial state
        if (compactState == null) {
            stopArmMotors();
            if (isCompactPose()) {
                compactState = CompactState.COMPACT_POSE;
            }
            else {
                boolean tooLow = armSubsystem.getArmLiftAngle() < CLEAR_FRAME_ARM_ANGLE;
                compactState = tooLow ? CompactState.PREPARING : CompactState.RETRACTING;
            }

            System.out.println("moveToCompactPose: initial state" + compactState);
        }

        // get into the compact pose
        switch (compactState) {

        case PREPARING: {
            if (!armSubsystem.isPincherAtCloseLimit()) {
                armSubsystem.setPincherSpeed(MAX_PINCHER_SPEED);
            }
            boolean done = moveArmLiftToAngle(CLEAR_FRAME_ARM_ANGLE);
            compactState = done ? CompactState.RETRACTING : CompactState.PREPARING;
            if (compactState != CompactState.PREPARING) {
                System.out.println("moveToCompactPose: change state from PREPARING to " + compactState);
            }
            break;
        }

        case RETRACTING: {
            boolean done = movePincherToEncoderCount(PINCHER_CLOSE_LIMIT_ENCODER_VALUE);
            done         = moveArmExtendToEncoderCount(0, ArmConstants.MAX_EXTEND_SPEED) && done;
            compactState = done ? CompactState.LOWERING : CompactState.RETRACTING;
            if (compactState != CompactState.RETRACTING) {
                System.out.println("moveToCompactPose: change state from RETRACTING to " + compactState);
            }
            break;
        }

        case LOWERING: {
            boolean done = moveArmLiftToAngle(0);
            compactState = done ? CompactState.COMPACT_POSE : CompactState.LOWERING;
            if (compactState != CompactState.LOWERING) {
                System.out.println("moveToCompactPose: change state from LOWERING to " + compactState);
            }
            break;
        }

        case COMPACT_POSE: {
            // done!
            System.out.println("Compact pose achieved");
            stopArmMotors();
            compactState = null; // ready to go again
            return true;
        }
        }
        return false;
    }

    private enum DriveWithPieceState {
        PREPARING, RETRACTING, FINALIZING_ANGLE, FINALIZING_EXTENT, IN_POSITION;
    }

    private DriveWithPieceState driveWithPieceState = null;

    protected final boolean moveToDriveWithPiecePose() {

        final Constants.GameConstants.GamePiece heldGamePiece = armSubsystem.getHeldGamePiece();
        if (heldGamePiece == Constants.GameConstants.GamePiece.NONE) {
            System.out.println("Attempting to move to a drive with game piece position but not holding a piece. Compacting.");
            return moveToCompactPose();
        }

        final Constants.ArmPosition target = Constants.ArmConstants.getDrivePosition(heldGamePiece);
        if (armSubsystem.isInPosition(target)) {
            stopArmMotors();
            return true;
        }
        else if (driveWithPieceState == null) {
            boolean tooLow = armSubsystem.getArmLiftAngle() < CLEAR_FRAME_ARM_ANGLE;
            driveWithPieceState = tooLow ? DriveWithPieceState.PREPARING : DriveWithPieceState.RETRACTING;
        }

        // get into the compact pose
        switch (driveWithPieceState) {

        case PREPARING: {
            if (moveArmLiftToAngle(CLEAR_FRAME_ARM_ANGLE)) {
                driveWithPieceState = DriveWithPieceState.RETRACTING;
                System.out.println("moveToDriveWithPiece: change state from PREPARING to " + compactState);
            }
            break;
        }

        case RETRACTING: {
            if (retractArm()) {
                driveWithPieceState = DriveWithPieceState.FINALIZING_ANGLE;
                System.out.println("moveToDriveWithPiece: change state from RETRACTING to " + compactState);
            }
            break;
        }

        case FINALIZING_ANGLE: {
            if (moveArmLiftToAngle(target.angle)) {
                driveWithPieceState = DriveWithPieceState.FINALIZING_EXTENT;
                System.out.println("moveToDriveWithPiece: change state from FINALIZE_ANGLE to " + compactState);
            }
            break;
        }

        case FINALIZING_EXTENT: {
            if (moveArmExtendToEncoderCount(target.extension, .5)) {
                driveWithPieceState = DriveWithPieceState.IN_POSITION;
                System.out.println("moveToDriveWithPiece: change state from FINALIZING_EXTENT to " + compactState);
            }
            break;
        }

        case IN_POSITION: {
            // done!
            System.out.println("Drive with piece pose achieved");
            stopArmMotors();
            driveWithPieceState = null; // ready to go again
            return true;
        }
        }
        return false;
    }
}

