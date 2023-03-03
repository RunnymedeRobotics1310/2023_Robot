package frc.robot.commands.arm;

import static frc.robot.Constants.ArmConstants.CLEAR_FRAME_LIFT_ENCODER_LOCATION;
import static frc.robot.Constants.ArmConstants.GROUND_PICKUP_EXTEND;
import static frc.robot.Constants.ArmConstants.GROUND_PICKUP_HEIGHT;
import static frc.robot.Constants.ArmConstants.GROUND_PICKUP_PINCHER_WIDTH;

import frc.robot.Constants.GameConstants.GamePiece;
import frc.robot.commands.operator.OperatorInput;
import frc.robot.subsystems.ArmSubsystem;

public class PickUpGroundCommand extends BaseArmCommand {

    private final ArmSubsystem  armSubsystem;
    private final OperatorInput operatorInput;

    // https://docs.google.com/document/d/1JzU-BzCXjGCwosouylmWGN83-x8lv-oPzklcXDqNN2U/edit#

    /**
     * Pick up a piece from the ground. This command will remain active until a piece is grasped or
     * it is interrupted or
     * cancelled.
     *
     * @param operatorInput the controllers
     * @param armSubsystem the arm subsystem
     */
    public PickUpGroundCommand(OperatorInput operatorInput, ArmSubsystem armSubsystem) {
        super(armSubsystem);

        this.armSubsystem  = armSubsystem;
        this.operatorInput = operatorInput;

        addRequirements(armSubsystem);

    }

    private enum State {

        MOVING_TO_COMPACT_POSE,
        COMPACT_POSE, // raise arm to safe height
        ARM_MOVING, // extend arm, open pincher and raise arm
        ARM_IN_POSITION, // arm is in the proper position for hoovering
        PIECE_DETECTED, // piece has been found
        PINCHERS_MOVING, // close pinchers
        PIECE_GRABBED // all done!
    }

    private State state = State.MOVING_TO_COMPACT_POSE;

    private void printStatus(String msg) {
        System.out.println("PickupGroundCommand: " + msg);
        System.out.println("PickupGroundCommand: State: " + state);
        printArmState();
    }

    @Override
    public void initialize() {
        printStatus("initialize");
        if (isCompactPose()) {
            state = State.COMPACT_POSE;
            stopArmMotors();
        }
        else {
            armSubsystem.setPincherSpeed(.25);
            armSubsystem.setArmExtendSpeed(0);
            armSubsystem.setArmLiftSpeed(0);
            state = State.MOVING_TO_COMPACT_POSE;
        }
    }

    @Override
    public void execute() {
        printStatus("execute"); // FIXME: This print will flood the logs.

        switch (state) {

        case MOVING_TO_COMPACT_POSE: {
            boolean done = moveToCompactPose();
            if (done) {
                state = State.COMPACT_POSE;
            }
            break;
        }

        case COMPACT_POSE: {
            // sets arm position to ground pickup
            boolean liftDone = moveArmLiftToEncoderCount(CLEAR_FRAME_LIFT_ENCODER_LOCATION, .15);
            if (liftDone) {
                state = State.ARM_MOVING;
            }
            break;
        }

        case ARM_MOVING: {
            boolean liftDone  = moveArmLiftToEncoderCount(GROUND_PICKUP_HEIGHT, .15);
            boolean extDone   = moveArmExtendToEncoderCount(GROUND_PICKUP_EXTEND, .5);
            boolean pinchDone = movePincherToEncoderCount(GROUND_PICKUP_PINCHER_WIDTH, .5);
            if (liftDone && extDone && pinchDone) {
                state = State.ARM_IN_POSITION;
            }
            break;
        }

        case ARM_IN_POSITION: {
            if (armSubsystem.isGamePieceDetected()) {
                state = State.PIECE_DETECTED;
            }
            break;
        }

        case PIECE_DETECTED:
        case PINCHERS_MOVING: {
            if (operatorInput.isPickUpCube()) {
                boolean pinchDone = movePincherToEncoderCount(GamePiece.CUBE.pincherEncoderCount, .5);
                state = pinchDone ? State.PIECE_GRABBED : State.PINCHERS_MOVING;
            }
            else if (operatorInput.isPickUpCone()) {
                boolean pinchDone = movePincherToEncoderCount(GamePiece.CONE.pincherEncoderCount, .5);
                state = pinchDone ? State.PIECE_GRABBED : State.PINCHERS_MOVING;
            }

            // FIXME: What happens when neither trigger is pressed?
            break;
        }

        default:
            System.out.println("PickupGroundCommand: Unknown state encountered " + state);
        }
    }

    @Override
    public boolean isFinished() {
        printStatus("finished"); // FIXME: This print will flood the logs.
        return state == State.PIECE_GRABBED && armSubsystem.getHeldGamePiece() != GamePiece.NONE;
    }

    @Override
    public void end(boolean interrupted) {
        printStatus("interrupted: " + interrupted);
        stopArmMotors();
    }
}
