package frc.robot.commands.arm;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmSubsystem;

import static frc.robot.Constants.ArmConstants.*;

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
     * @param targetCount the count to get to
     * @param speed the absolute value of the speed at which you should move
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
     * @param targetCount the count to get to
     * @param speed the absolute value of the speed at which you should move
     * @return true if at the desired location, false if still moving to that point
     */
    protected final boolean moveArmExtendToEncoderCount(double targetCount, double speed) {
        double absSpd = Math.abs(speed);
        double gap    = armSubsystem.getArmExtendEncoder() - targetCount;
        if (Math.abs(gap) > PINCHER_MOTOR_TOLERANCE) {
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
     * @param targetCount the count to get to
     * @param speed the absolute value of the speed at which you should move
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
            stopArmMotors();
            if (isCompactPose()) {
                compactState = CompactState.COMPACT_POSE;
            }
            else {
                boolean tooLow = armSubsystem.getArmLiftEncoder() < CLEAR_FRAME_LIFT_ENCODER_LOCATION;
                compactState = tooLow ? CompactState.PREPARING : CompactState.RETRACTING;
            }
        }

        // get into the compact pose
        switch (compactState) {
        case PREPARING: {
            if (!armSubsystem.isPincherAtCloseLimit()) {
                armSubsystem.setPincherSpeed(.5);
            }
            boolean done = moveArmLiftToEncoderCount(CLEAR_FRAME_LIFT_ENCODER_LOCATION, .3);
            compactState = done ? CompactState.RETRACTING : CompactState.PREPARING;
            break;
        }
        case RETRACTING: {
            boolean done = movePincherToEncoderCount(PINCHER_CLOSE_LIMIT_ENCODER_VALUE, .5);
            done         = moveArmExtendToEncoderCount(0, .3) && done;
            compactState = done ? CompactState.LOWERING : CompactState.RETRACTING;
            break;
        }
        case LOWERING: {
            boolean done = moveArmLiftToEncoderCount(0, .1);
            compactState = done ? CompactState.COMPACT_POSE : CompactState.LOWERING;
            break;
        }
        case COMPACT_POSE: {
            // done!
            return true;
        }
        }
        return false;
    }
}

