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
        ARM_RETRACTING, COMPAT_POSE, ARM_MOVING, ARM_IN_POSITION, PIECE_DETECTED, PINCHERS_MOVING, PIECE_GRABBED
    }

    private State state = State.ARM_RETRACTING;

    private void printStatus(String msg) {
        System.out.println("PickupGroundCommand: " + msg);
        System.out.println("PickupGroundCommand: State: " + state);
        printState();
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
            state = State.ARM_RETRACTING;
        }
    }

    private final double CLEAR_FRAME_LIFT_ENCODER_LOCATION = 3; // todo: fixme - get real value

    @Override
    public void execute() {
        printStatus("execute");

        switch (state) {
        case ARM_RETRACTING: {
            if (armSubsystem.isArmRetracted() && armSubsystem.isPincherAtCloseLimit() && armSubsystem.isArmDown()) {
                state = State.COMPAT_POSE;
                stopArmMotors();
            }
            else {
                armSubsystem.setPincherSpeed(.25);
                armSubsystem.setArmExtendSpeed(0);
                armSubsystem.setArmLiftSpeed(0);
                state = State.ARM_RETRACTING;
            }



            if (armSubsystem.isArmRetracted()) {
                state = State.COMPAT_POSE;
                armSubsystem.setArmExtendSpeed(0);
            }
            else {
                armSubsystem.setArmExtendSpeed(-0.1);
                state = State.ARM_RETRACTING;
            }
        }
        case COMPAT_POSE: {
            // sets arm position to ground pickup
            boolean moving = moveArmLiftToEncoderCount(GROUND_PICKUP_HEIGHT, .25);

            // don't extend arm or open pinchers if we haven't extended raised the arm above the frame
            if (armSubsystem.getArmLiftEncoder() >= CLEAR_FRAME_LIFT_ENCODER_LOCATION) {

                moving = moving || moveArmExtendToEncoderCount(CLEAR_FRAME_LIFT_ENCODER_LOCATION, .25);
                moving = moving || movePincherToEncoderCount(0, -.25);
            }

            state = moving ? State.ARM_MOVING : State.ARM_IN_POSITION;
        }
        case ARM_IN_POSITION: {
            if (armSubsystem.isGamePieceDetected()) {
                state = State.PIECE_DETECTED;
            }
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
