package frc.robot.commands.arm;

import frc.robot.Constants.ArmConstants;
import frc.robot.subsystems.ArmSubsystem;

public class CompactCommand_SafeForGamePiece extends BaseArmCommand {

    private enum State {
        CLEAR_FRAME, RETRACT, DROP_PIECE, LOWER
    };


    private State state = State.CLEAR_FRAME;

    public CompactCommand_SafeForGamePiece(ArmSubsystem armSubsystem) {
        super(armSubsystem);
        addRequirements(armSubsystem);
    }

    @Override
    public void initialize() {

        double armAngle             = armSubsystem.getArmLiftAngle();
        double armExtensionPosition = armSubsystem.getArmExtendEncoder();

        if (armAngle < ArmConstants.CLEAR_FRAME_ARM_ANGLE
            && armExtensionPosition > ArmConstants.MAX_ARM_EXTEND_INSIDE_FRAME) {

            state = State.CLEAR_FRAME;
        }
        else {
            state = State.RETRACT;
        }

        logCommandStart("Starting State " + state);
    }

    @Override
    public void execute() {

        double  armAngle             = armSubsystem.getArmLiftAngle();
        double  armExtensionPosition = armSubsystem.getArmExtendEncoder();
        boolean pincherInsideFrame   = armSubsystem.isPincherInsideFrame();

        switch (state) {

        case CLEAR_FRAME:

            /*
             * Special logic to make sure the arm comes up over the frame when retracting.
             */
            if (armAngle < ArmConstants.CLEAR_FRAME_ARM_ANGLE
                && armExtensionPosition > ArmConstants.MAX_ARM_EXTEND_INSIDE_FRAME) {

                /*
                 * If the arm is not inside the frame, and the angle is below the clear frame angle,
                 * then raise the arm to clear the frame
                 */

                // Move the pincher appropriately to save time.
                if (armSubsystem.isGamePieceDetected()) {
                    openPincher();
                }
                else {
                    movePincherInsideFrame();
                }

                // Ensure the arm is higher than the clear frame angle
                moveArmLiftToAngle(ArmConstants.CLEAR_FRAME_ARM_ANGLE + ArmConstants.ARM_LIFT_ANGLE_TOLERANCE_DEGREES + 2);

            }
            else {
                armSubsystem.setArmLiftSpeed(0);
                logStateTransition("CLEAR_FRAME -> RETRACT");
                state = State.RETRACT;
            }

            break;

        case RETRACT:

            /*
             * The arm is now clearing the frame or is already inside the frame
             */
            boolean armRetracted = retractArm();

            if (armSubsystem.isGamePieceDetected()) {
                openPincher();
            }
            else {
                movePincherInsideFrame();
            }

            if (armRetracted) {
                logStateTransition("RETRACT -> DROP_PIECE");
                state = State.DROP_PIECE;
            }
            break;

        case DROP_PIECE:

            // Keep the arm retracted
            retractArm();

            if (armSubsystem.isGamePieceDetected()) {

                openPincher();

                // Raise the arm until a game piece is not detected.
                moveArmLiftToAngle(ArmConstants.SCORE_MIDDLE_CONE_POSITION.angle);

            }
            else {

                logStateTransition("DROP_PIECE -> LOWER");
                state = State.LOWER;
            }
            break;

        case LOWER:

            // Keep the arm retracted
            retractArm();

            pincherInsideFrame = movePincherInsideFrame();

            if (pincherInsideFrame) {
                // Lower the arm to the bottom
                lowerArmToBottom();
            }

            break;
        }
    }

    @Override
    public boolean isFinished() {

        long timeout = 3000;
        if (System.currentTimeMillis() - initializeTime > timeout) {
            setFinishReason("Command timed out after " + timeout + "ms");
            return true;
        }


        if (isCompactPose()) {
            setFinishReason("in Compact Pose");
            return true;
        }

        return false;
    }

    @Override
    public void end(boolean interrupted) {

        stopArmMotors();

        logCommandEnd(interrupted);
    }
}
