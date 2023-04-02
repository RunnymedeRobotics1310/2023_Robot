package frc.robot.commands.arm;

import frc.robot.Constants.ArmConstants;
import frc.robot.subsystems.ArmSubsystem;

public class CompactCommand extends BaseArmCommand {

    private enum State {
        CLEAR_FRAME, RETRACT, LOWER
    };


    private State state = State.CLEAR_FRAME;

    public CompactCommand(ArmSubsystem armSubsystem) {
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
        commandStart = new System.currentTimeMillis();

        logCommandStart("Starting State " + state);
    }

    @Override
    public void execute() {

        double armAngle             = armSubsystem.getArmLiftAngle();
        double armExtensionPosition = armSubsystem.getArmExtendEncoder();

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

                // Save time by closing the pincher at the same time
                movePincherInsideFrame();

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
            boolean pincherInsideFrame = movePincherInsideFrame();

            if (armRetracted && pincherInsideFrame) {
                logStateTransition("RETRACT -> LOWER");
                state = State.LOWER;
            }
            break;

        case LOWER:

            // Keep the arm retracted and do not let the pinchers outside the frame
            retractArm();
            movePincherInsideFrame();

            // Lower the arm to the bottom
            lowerArmToBottom();
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
