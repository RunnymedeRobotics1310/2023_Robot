package frc.robot.commands.arm;

import frc.robot.Constants.ArmConstants;
import frc.robot.subsystems.ArmSubsystem;

public class CompactCommand2 extends BaseArmCommand {

    public CompactCommand2(ArmSubsystem armSubsystem) {
        super(armSubsystem);
        addRequirements(armSubsystem);
    }

    private void printStatus(String msg) {
        System.out.println("Compact Command: " + msg);
        printArmState();
    }

    @Override
    public void initialize() {
        printStatus("initialize");
    }

    @Override
    public void execute() {

        double  armAngle             = armSubsystem.getArmLiftAngle();
        double  armExtensionPosition = armSubsystem.getArmExtendEncoder();

        boolean armRetracted         = false;
        boolean pincherInsideFrame   = false;

        /*
         * Special logic to make sure the arm comes up over the frame when retracting.
         */
        if (armAngle < ArmConstants.CLEAR_FRAME_ARM_ANGLE) {

            // If the arm is inside the frame, then retract the arm and
            // close the pincher to fit inside the frame.
            if (armExtensionPosition < ArmConstants.MAX_ARM_EXTEND_INSIDE_FRAME) {

                armRetracted       = retractArm();
                pincherInsideFrame = movePincherInsideFrame();

                if (armRetracted && pincherInsideFrame) {
                    lowerArmToBottom();
                }
            }
            else {
                /*
                 * If the arm is not inside the frame, and the angle is below the clear frame angle,
                 * then raise the arm to clear the frame
                 */

                // Save time by closing the pincher at the same time
                movePincherInsideFrame();

                // Ensure the arm is higher than the clear frame angle
                moveArmLiftToAngle(ArmConstants.CLEAR_FRAME_ARM_ANGLE + ArmConstants.ARM_LIFT_ANGLE_TOLERANCE_DEGREES + 2);
            }

            return;
        }

        /*
         * The arm is now clearing the frame.
         */

        armRetracted       = retractArm();
        pincherInsideFrame = movePincherInsideFrame();

        if (armRetracted && pincherInsideFrame) {
            lowerArmToBottom();
        }
    }

    @Override
    public boolean isFinished() {
        return isCompactPose();
    }

    @Override
    public void end(boolean interrupted) {
        if (interrupted) {
            printStatus("End. Interrupted");
        }
        else {
            printStatus("End. Not interrupted");
        }
        stopArmMotors();
    }
}
