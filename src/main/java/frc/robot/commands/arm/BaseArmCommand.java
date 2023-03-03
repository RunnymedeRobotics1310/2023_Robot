package frc.robot.commands.arm;

import static frc.robot.Constants.ArmConstants.ARM_EXTEND_MOTOR_TOLERANCE;
import static frc.robot.Constants.ArmConstants.ARM_LIFT_MOTOR_TOLERANCE;
import static frc.robot.Constants.ArmConstants.CLEAR_FRAME_LIFT_ENCODER_LOCATION;
import static frc.robot.Constants.ArmConstants.MAX_PINCHER_SPEED;
import static frc.robot.Constants.ArmConstants.PINCHER_CLOSE_LIMIT_ENCODER_VALUE;
import static frc.robot.Constants.ArmConstants.PINCHER_MOTOR_TOLERANCE;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmSubsystem;

abstract class BaseArmCommand extends CommandBase {

    private final ArmSubsystem armSubsystem;

    protected BaseArmCommand(ArmSubsystem armSubsystem) {
        this.armSubsystem = armSubsystem;
    }

    protected final void printArmState() {
        System.out.println("BaseArmCommand: armSubsystem.getHeldGamePiece: " + armSubsystem.getHeldGamePiece());
        System.out.println("BaseArmCommand: Arm height: " + armSubsystem.getArmLiftEncoder());
        System.out.println("BaseArmCommand: Arm extent: " + armSubsystem.getArmExtendEncoder());
        System.out.println("BaseArmCommand: Pincher: " + armSubsystem.getPincherEncoder());
    }

    protected final void stopArmMotors() {
        armSubsystem.setArmExtendSpeed(0);
        armSubsystem.setPincherSpeed(0);
        armSubsystem.setArmLiftSpeed(0);
    }

    /**
     * Move the motor to a specified encoder count
     *
     * @param targetCount The count to get to
     * @param speed Speed that the motor should move to the target. Direction (sign) will be ignored and computed automatically.
     * @return true if at the desired location, false if still moving to that point
     */
    protected final boolean moveArmLiftToEncoderCount(double targetCount, double speed) {
        double absSpd = Math.abs(speed);
        double gap    = armSubsystem.getArmLiftEncoder() - targetCount;
        if (Math.abs(gap) > ARM_LIFT_MOTOR_TOLERANCE) {
            armSubsystem.setArmLiftSpeed(gap > 0 ? -absSpd : absSpd);
            return false;
        }
        else {
            armSubsystem.setArmLiftSpeed(0);
            return true;
        }
    }

    /**
     * Move the motor to a specified encoder count
     *
     * @param targetCount The count to get to
     * @param speed Speed that the motor should move to the target. Direction (sign) will be ignored and computed automatically.
     * @return true if at the desired location, false if still moving to that point
     */
    protected final boolean moveArmExtendToEncoderCount(double targetCount, double speed) {
        double absSpd = Math.abs(speed);
        double gap    = armSubsystem.getArmExtendEncoder() - targetCount;
        if (Math.abs(gap) > ARM_EXTEND_MOTOR_TOLERANCE) {
            armSubsystem.setArmExtendSpeed(gap > 0 ? -absSpd : absSpd);
            return false;
        }
        else {
            armSubsystem.setArmExtendSpeed(0);
            return true;
        }
    }

    /**
     * Move the motor to a specified encoder count
     *
     * @param targetCount The count to get to
     * @param speed Speed that the motor should move to the target. Direction (sign) will be ignored and computed automatically.
     * @return true if at the desired location, false if still moving to that point
     */
    protected final boolean movePincherToEncoderCount(double targetCount, double speed) {
        double absSpd = Math.abs(speed);
        double gap    = armSubsystem.getPincherEncoder() - targetCount;
        if (Math.abs(gap) > PINCHER_MOTOR_TOLERANCE) {
            armSubsystem.setPincherSpeed(gap > 0 ? -absSpd : absSpd);
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
        return armSubsystem.isArmDown() && armSubsystem.isArmRetracted() && armSubsystem.isPincherAtCloseLimit();
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
            System.out.println("moveToCompactPose starting");
            stopArmMotors();
            if (isCompactPose()) {
                compactState = CompactState.COMPACT_POSE;
            }
            else {
                boolean tooLow = armSubsystem.getArmLiftEncoder() < CLEAR_FRAME_LIFT_ENCODER_LOCATION;
                compactState = tooLow ? CompactState.PREPARING : CompactState.RETRACTING;
            }
        }
        else {
            // System.out.println("Moving to compact pose. Current state: " + compactState);
        }

        // get into the compact pose
        switch (compactState) {

        case PREPARING: {
            if (!armSubsystem.isPincherAtCloseLimit()) {
                armSubsystem.setPincherSpeed(.5);
            }
            boolean liftDone = moveArmLiftToEncoderCount(CLEAR_FRAME_LIFT_ENCODER_LOCATION, .3);
            if (liftDone) {
                compactState = CompactState.RETRACTING;
            }
            break;
        }

        case RETRACTING: {
            boolean pinchDone  = movePincherToEncoderCount(PINCHER_CLOSE_LIMIT_ENCODER_VALUE, .5);
            boolean extendDone = moveArmExtendToEncoderCount(0, .3);
            if (pinchDone && extendDone) {
                compactState = CompactState.LOWERING;
            }
            break;
        }

        case LOWERING: {
            boolean liftDone = moveArmLiftToEncoderCount(0, .1);
            if (liftDone) {
                compactState = CompactState.COMPACT_POSE;
            }
            break;
        }

        case COMPACT_POSE: {
            System.out.println("Compact pose achieved");
            stopArmMotors();
            compactState = null; // ready to go again
            return true;
        }
        }
        return false;
    }
}

