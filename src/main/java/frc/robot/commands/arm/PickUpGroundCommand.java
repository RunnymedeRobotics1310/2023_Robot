package frc.robot.commands.arm;

import static frc.robot.Constants.ArmConstants.*;
import static frc.robot.Constants.GameConstants.GamePiece.*;

import frc.robot.Constants.GameConstants.GamePiece;
import frc.robot.commands.operator.OperatorInput;
import frc.robot.subsystems.ArmSubsystem;

public class PickUpGroundCommand extends BaseArmCommand {

    private final ArmSubsystem  armSubsystem;
    private final OperatorInput operatorInput;
    private GamePiece           pickupTarget;

    // https://docs.google.com/document/d/1JzU-BzCXjGCwosouylmWGN83-x8lv-oPzklcXDqNN2U/edit#

    /**
     * Pick up a piece from the ground. This command will remain active until a piece is grasped or it is interrupted or
     * cancelled.
     *
     * @param armSubsystem the arm subsystem
     * @param operatorInput optional operator input object which allows the operator to change the selected game piece after the
     * command has started. If not specified, the initialPickupTarget will remain the active pickup target for the duration of the
     * command.
     * @param initialPickupTarget the type of game piece to be picked up - unless later overridden by the operator
     */
    public PickUpGroundCommand(ArmSubsystem armSubsystem, OperatorInput operatorInput, GamePiece initialPickupTarget) {
        super(armSubsystem);

        this.armSubsystem  = armSubsystem;
        this.operatorInput = operatorInput;
        this.pickupTarget  = initialPickupTarget;

        addRequirements(armSubsystem);

    }

    private enum State {

        MOVING_TO_COMPACT_POSE,
        COMPACT_POSE, // raise arm to safe height
        ARM_MOVING, // extend arm, open pincher and raise arm
        ARM_IN_POSITION, // arm is in the proper position for hoovering
        PIECE_DETECTED, // piece has been found
        PINCHERS_MOVING, // close pinchers
        PIECE_GRABBED, // all done!
        CANCELLED // aborted
    }

    private State state = State.MOVING_TO_COMPACT_POSE;

    private void printStatus(String msg) {
        System.out.println("PickupGroundCommand: " + msg);
        System.out.println("PickupGroundCommand: Target: " + pickupTarget);
        System.out.println("PickupGroundCommand: State: " + state);
        printArmState();
    }

    private boolean isGroundPickupPose() {
        if (!armSubsystem.isArmAtLiftAngle(GROUND_PICKUP_POSITION.angle))
            return false;
        double ext = armSubsystem.getArmExtendEncoder();
        if (Math.abs(ext - GROUND_PICKUP_POSITION.extension) > ARM_EXTEND_POSITION_TOLERANCE)
            return false;
        if (!armSubsystem.isPincherOpen())
            return false;
        return true;
    }

    @Override
    public void initialize() {
        printStatus("initialize");
        if (isCompactPose()) {
            state = State.COMPACT_POSE;
            stopArmMotors();
        }
        else if (isGroundPickupPose()) {
            state = State.ARM_IN_POSITION;
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

        switch (state) {

        case MOVING_TO_COMPACT_POSE: {
            boolean compactDone = moveToCompactPose();
            if (compactDone) {
                state = State.COMPACT_POSE;
            }
            break;
        }

        case COMPACT_POSE: {
            // sets arm position to ground pickup
            boolean liftDone = moveArmLiftToAngle(CLEAR_FRAME_ARM_ANGLE, .15);
            if (liftDone) {
                state = State.ARM_MOVING;
            }
            break;
        }

        case ARM_MOVING: {
            // if ground pickup location is too low, override it with the location that will ensure that we clear the frame.
            boolean liftDone  = moveArmLiftToAngle(
                Double.max(CLEAR_FRAME_ARM_ANGLE, GROUND_PICKUP_POSITION.angle), .15);
            boolean extDone   = moveArmExtendToEncoderCount(GROUND_PICKUP_POSITION.extension, .5);
            boolean pinchDone = openPincher();
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
            if (operatorInput != null) {
                // check for operator override. If both selected, cube wins. If none selected, command is cancelled.
                if (operatorInput.isPickUpCone()) {
                    if (pickupTarget == CUBE) {
                        System.out.println("Changing pickup target from CUBE to CONE based on operator input");
                        pickupTarget = CONE;
                    }
                }
                else if (operatorInput.isPickUpCube()) {
                    if (pickupTarget == CONE) {
                        System.out.println("Changing pickup target from CONE to CUBE based on operator input");
                        pickupTarget = CUBE;
                    }
                }
                else {
                    pickupTarget = NONE;
                    state        = State.CANCELLED;
                    break;
                }
            }
            boolean pinchDone = movePincherToEncoderCount(pickupTarget.pincherEncoderCount, .5);
            state = pinchDone ? State.PIECE_GRABBED : State.PINCHERS_MOVING;
            break;
        }

        case CANCELLED: {
            // note when this occurs the robot remains in this state.
            System.out.println("No pickup target specified by operator. Cancelling command.");
            break;
        }

        default:
            printStatus("unknown state encountered");
        }
    }

    @Override
    public boolean isFinished() {
        return state == State.CANCELLED || (state == State.PIECE_GRABBED && armSubsystem.getHeldGamePiece() != NONE);
    }

    @Override
    public void end(boolean interrupted) {
        printStatus("end. Interrupted? " + interrupted);
        stopArmMotors();
    }
}
