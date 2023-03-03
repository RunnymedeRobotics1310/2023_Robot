package frc.robot.commands.arm;

import frc.robot.Constants.GameConstants.GamePiece;
import frc.robot.commands.operator.OperatorInput;
import frc.robot.subsystems.ArmSubsystem;

import static frc.robot.Constants.ArmConstants.*;

// FIXME: THIS IS IN PROGRESS BUT COMPILING. NOT READY TO RUN YET
public class PickUpGroundCommand extends BaseArmCommand {

    private final ArmSubsystem  armSubsystem;
    private final OperatorInput operatorInput;

    // https://docs.google.com/document/d/1JzU-BzCXjGCwosouylmWGN83-x8lv-oPzklcXDqNN2U/edit#

    /**
     * Pick up a piece from the ground. This command will remain active until a piece is grasped or it is interrupted or
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

        ARM_PREPARING_FOR_RETRACTION, // make sure arm is high enough to clear bumper
        ARM_RETRACTING, // pull the arm in and close the pincher
        ARM_LOWERING_TO_COMPACT, // lower arm to compact post
        COMPAT_POSE, // raise arm to safe height
        ARM_MOVING, // extend arm, open pincher and raise arm
        ARM_IN_POSITION, // arm is in the proper position for hoovering
        PIECE_DETECTED, // piece has been found
        PINCHERS_MOVING, // close pinchers
        PIECE_GRABBED // all done!
    }

    private State state = State.ARM_PREPARING_FOR_RETRACTION;

    private void printStatus(String msg) {
        System.out.println("PickupGroundCommand: " + msg);
        System.out.println("PickupGroundCommand: State: " + state);
        printArmState();
    }

    @Override
    public void initialize() {
        printStatus("initialize");
        // dump arm data
        if (armSubsystem.isArmRetracted() && armSubsystem.isPincherAtCloseLimit() && armSubsystem.isArmDown()) {
            state = State.COMPAT_POSE;
            stopArmMotors();
        }
        else {
            armSubsystem.setPincherSpeed(.25);
            armSubsystem.setArmExtendSpeed(0);
            armSubsystem.setArmLiftSpeed(0);
            state = State.ARM_PREPARING_FOR_RETRACTION;
        }
    }

    private final double CLEAR_FRAME_LIFT_ENCODER_LOCATION = 3; // todo: fixme - get real value

    @Override
    public void execute() {
        printStatus("execute");

        switch (state) {
        case ARM_PREPARING_FOR_RETRACTION: {
            if (armSubsystem.isArmRetracted() && armSubsystem.isPincherAtCloseLimit() && armSubsystem.isArmDown()) {
                stopArmMotors();
                state = State.COMPAT_POSE;
            }
            else {
                boolean moving = moveArmLiftToEncoderCount(CLEAR_FRAME_LIFT_ENCODER_LOCATION, .25);
                state = moving ? State.ARM_PREPARING_FOR_RETRACTION : State.ARM_RETRACTING;
            }
            break;
        }
        case ARM_RETRACTING: {
            armSubsystem.setPincherSpeed(.25);
            armSubsystem.setArmLiftSpeed(0);
            if (armSubsystem.isArmRetracted()) {
                armSubsystem.setArmExtendSpeed(0);
                state = State.ARM_LOWERING_TO_COMPACT;
            }
            else {
                armSubsystem.setArmExtendEncoder(-.25);
            }
            break;
        }
        case ARM_LOWERING_TO_COMPACT: {
            if (armSubsystem.isArmDown()) {
                stopArmMotors();
                state = State.COMPAT_POSE;
            }
            else {
                moveArmLiftToEncoderCount(0, .25);
            }
            break;
        }
        case COMPAT_POSE: {
            // sets arm position to ground pickup
            boolean moving = moveArmLiftToEncoderCount(CLEAR_FRAME_LIFT_ENCODER_LOCATION, .25);
            state = moving ? State.COMPAT_POSE : State.ARM_MOVING;
            break;
        }
        case ARM_MOVING: {
            boolean moving = moveArmLiftToEncoderCount(GROUND_PICKUP_HEIGHT, .25);
            moving = moving || moveArmExtendToEncoderCount(CLEAR_FRAME_LIFT_ENCODER_LOCATION, .25);
            moving = moving || movePincherToEncoderCount(0, -.25);
            state  = moving ? State.ARM_MOVING : State.ARM_IN_POSITION;
            break;
        }
        case ARM_IN_POSITION: {
            if (armSubsystem.isGamePieceDetected()) {
                state = State.PIECE_DETECTED;
            }
            break;
        }
        case PIECE_DETECTED: {
            if (operatorInput.isPickUpCube()) {
                movePincherToEncoderCount(GamePiece.CUBE.pincherEncoderCount, .25);
                state = State.PINCHERS_MOVING;
            }
            else if (operatorInput.isPickUpCone()) {
                movePincherToEncoderCount(GamePiece.CONE.pincherEncoderCount, .25);
                state = State.PINCHERS_MOVING;
            }
            else {
                armSubsystem.setPincherSpeed(0);
                state = State.PIECE_GRABBED;
            }
            break;
        }
        }

    }

    @Override
    public boolean isFinished() {
        printStatus("finished");
        return state == State.PIECE_GRABBED && armSubsystem.getHeldGamePiece() != GamePiece.NONE;
    }

    @Override
    public void end(boolean interrupted) {
        printStatus("interrupted: " + interrupted);
        stopArmMotors();
    }
}
