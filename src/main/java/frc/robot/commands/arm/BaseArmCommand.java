package frc.robot.commands.arm;

import static frc.robot.Constants.ArmConstants.*;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.ArmSubsystem;

abstract class BaseArmCommand extends CommandBase {

    private final ArmSubsystem armSubsystem;

    protected BaseArmCommand(ArmSubsystem armSubsystem) {
        this.armSubsystem = armSubsystem;
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
     * Move the motor to a specified encoder count
     *
     * @param targetAngle the angle in degrees
     * @return true if at the desired location, false if still moving to that point
     */
    protected final boolean moveArmLiftToAngle(double targetAngle) {
        armSubsystem.setArmLiftPidEnabled(true);
        armSubsystem.moveArmToAngle(targetAngle);
        return armSubsystem.isArmAtLiftAngle(targetAngle);
    }

    /**
     * Move the motor to a specified encoder count
     *
     * @param targetCount the count to get to
     * @param speed the absolute value of the speed at which you should move
     * @return true if at the desired location, false if still moving to that point
     */
    protected final boolean moveArmExtendToEncoderCount(double targetCount, double speed) {

        SmartDashboard.putBoolean("Arm Extension at target", armSubsystem.isAtExtendPosition(targetCount));

        if (armSubsystem.isAtExtendPosition(targetCount)) {
            armSubsystem.setArmExtendSpeed(0);
            return true;
        }

        double absSpd = Math.abs(speed);
        double gap    = armSubsystem.getArmExtendEncoder() - targetCount;

        armSubsystem.setArmExtendSpeed(gap > 0 ? -absSpd : absSpd);

        return false;
    }

    /**
     * Move the motor to a specified encoder count
     *
     * @param targetCount the count to get to
     * @return true if at the desired location, false if still moving to that point
     */
    protected final boolean movePincherToEncoderCount(double targetCount) {
        double gap = armSubsystem.getPincherEncoder() - targetCount;
        if (Math.abs(gap) > PINCHER_POSITION_TOLERANCE) {
            armSubsystem.setPincherSpeed(gap > 0 ? -MAX_PINCHER_SPEED : MAX_PINCHER_SPEED);
            return false;
        }
        else {
            armSubsystem.setPincherSpeed(0);
            return true;
        }
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
            && armSubsystem.isArmRetracted() /* && armSubsystem.isPincherAtCloseLimit() */; // TODO: URGENT: FIXME - restore this
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
            done         = moveArmExtendToEncoderCount(0, .3) && done;
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
            boolean done = moveArmLiftToAngle(CLEAR_FRAME_ARM_ANGLE);
            driveWithPieceState = done ? DriveWithPieceState.RETRACTING : DriveWithPieceState.PREPARING;
            if (driveWithPieceState != DriveWithPieceState.PREPARING) {
                System.out.println("moveToDriveWithPiece: change state from PREPARING to " + compactState);
            }
            break;
        }

        case RETRACTING: {
            boolean done = moveArmExtendToEncoderCount(0, .5);
            driveWithPieceState = done ? DriveWithPieceState.FINALIZING_ANGLE : DriveWithPieceState.RETRACTING;
            if (driveWithPieceState != DriveWithPieceState.RETRACTING) {
                System.out.println("moveToDriveWithPiece: change state from RETRACTING to " + compactState);
            }
            break;
        }

        case FINALIZING_ANGLE: {
            boolean done = moveArmLiftToAngle(target.angle);
            driveWithPieceState = done ? DriveWithPieceState.FINALIZING_EXTENT : DriveWithPieceState.FINALIZING_ANGLE;
            if (driveWithPieceState != DriveWithPieceState.FINALIZING_ANGLE) {
                System.out.println("moveToDriveWithPiece: change state from FINALIZE_ANGLE to " + compactState);
            }
            break;
        }

        case FINALIZING_EXTENT: {
            boolean done = moveArmExtendToEncoderCount(target.extension, .5);
            driveWithPieceState = done ? DriveWithPieceState.IN_POSITION : DriveWithPieceState.FINALIZING_EXTENT;
            if (driveWithPieceState != DriveWithPieceState.FINALIZING_EXTENT) {
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

